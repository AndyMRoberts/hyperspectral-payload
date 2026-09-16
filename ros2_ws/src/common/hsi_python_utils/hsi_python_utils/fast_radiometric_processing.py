
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
import tqdm as tqdm

def resolve_context(context_dir):
    context_dir = Path(context_dir)
    return {
        "calibration_xml": next((context_dir / "calibration_file").glob("*.xml")),
        "dark_reference_xmls": sorted(
            (context_dir / "dark_references").glob("dark_reference*.raw.xml")
        ),
        "white_non_uniformity_xml": next(
            (context_dir / "non_uniformity").glob("white_reference*.raw.xml")
        ),
        "dark_non_uniformity_xml": next(
            (context_dir / "non_uniformity").glob("dark_reference*.raw.xml")
        ),
        "white_reference_xml": next(
            (context_dir / "white_reference").glob("white_reference*.raw.xml")
        ),
    }

def load_roi_frame(xml_path):
    root = ET.parse(xml_path).getroot()
    fmt = root.find("data_format")
    nr_rows = int(fmt.get("nr_rows"))
    nr_cols = int(fmt.get("nr_cols"))
    int_time_ms = float(root.find("data_info").get("integration_time_ms"))
    raw_path = str(xml_path).replace(".raw.xml", ".raw")
    data = np.fromfile(raw_path, dtype=np.float32).reshape(nr_rows, nr_cols)
    return data, int_time_ms

class FastProcessPipeline():
    def __init__(self, context_dir, white_panel_reflectance_path=None, matrix_type=None, median_blur=False, white_ref_correction=1):
        """
        Does all the preparation to perfom fast processing.
        """
        self.median_blur = median_blur
        if matrix_type == None:
            self.USE_CORRECTION_MATRIX = False
        else:
            self.USE_CORRECTION_MATRIX = True
        self.MATRIX_TYPE = matrix_type
        self.ctx = resolve_context(context_dir)
        self.mosaic_size, self.mosaic_indices, self.wavelengths_nm, self.correction_matrix = self.parse_calibration_bands(
            self.ctx["calibration_xml"]
        )
        self.white_non_uniformity, self.integration_time = load_roi_frame(self.ctx["white_non_uniformity_xml"])
        self.dark_non_uniformity, _ = load_roi_frame(self.ctx["dark_non_uniformity_xml"])
        self.white_ref_correction = white_ref_correction
        self.reference_panel_spectrum = self.load_reference_reflectance(white_panel_reflectance_path, self.wavelengths_nm)
        self.get_gain_and_bias()

    def get_info(self):
        return self.ctx


    def get_gain_and_bias(self):
        # create gain and bias, correction matrix to be applied as final step in other method
        denom = np.subtract(self.white_non_uniformity*self.white_ref_correction, self.dark_non_uniformity)
        eps = max(1e-3, 1e-6 * float(np.median(denom[denom > 0]))) if np.any(denom > 0) else 1e-3
        self.gain = 1.0 / np.maximum(denom, eps)
        self.bias = -np.multiply(self.dark_non_uniformity, self.gain) 
        

    def fast_process_to_reflectance(self, raw_scene_frame):
        """
        fastest processing, applying pre-calulated gain and bias to raw acquired scene frame.
        """
        # Apply gain and bias to cube
        scene_reflectance_frame = np.add(np.multiply(raw_scene_frame, self.gain), self.bias)  
        # frame to cube
        scene_reflectance_cube = self.demosaic_to_cube(scene_reflectance_frame, self.mosaic_size, self.mosaic_indices, self.correction_matrix)
        # finally apply reference spectrum adjustment
        # scene_reflectance_cube = scene_reflectance_cube @ self.reference_panel_spectrum
        scene_reflectance_cube = np.multiply(scene_reflectance_cube, self.reference_panel_spectrum )
        if self.median_blur:
            scene_reflectance_cube = self.median_filter_cube(scene_reflectance_cube)
        return scene_reflectance_cube

    def median_filter_cube(self, cube, kernel_size=3):
        """Apply spatial median filtering to each spectral band (matches imec kernel size 3)."""
        # cv2.medianBlur only supports float32 (not float64); reflectance scaling promotes to float64.
        cube = np.asarray(cube, dtype=np.float32)
        filtered = np.empty_like(cube)
        for band_idx in range(cube.shape[2]):
            filtered[:, :, band_idx] = cv2.medianBlur(cube[:, :, band_idx], kernel_size)
        return filtered

    
    def save_envi_reflectance(self, cube, output_hdr_path):
        cube = np.asarray(cube, dtype=np.float32)
        lines, samples, bands = cube.shape
        output_hdr_path = Path(output_hdr_path)
        output_hdr_path.parent.mkdir(parents=True, exist_ok=True)
        binary_path = output_hdr_path.with_suffix(".raw")
        cube.tofile(binary_path)

        wl_formatted = ", ".join(f"{w:.6f}" for w in self.wavelengths_nm)
        hdr_lines = [
            "ENVI",
            f"samples = {samples}",
            f"lines = {lines}",
            f"bands = {bands}",
            "header offset = 0",
            "file type = ENVI Standard",
            "data type = 4",
            "interleave = bip",
            "byte order = 0",
            "wavelength units = nm",
            f"wavelength = {{{wl_formatted}}}",
            "reflectance data = 1",
        ]
        if self.integration_time:
            hdr_lines.insert(1, f"acquisition time = {self.integration_time}")
        output_hdr_path.write_text("\n".join(hdr_lines) + "\n")
        return binary_path


    def load_reference_reflectance(self, reference_reflectance_path, target_wavelengths_nm):
        if reference_reflectance_path != None:
            df = pd.read_csv(reference_reflectance_path)
            lookup = dict(zip(df['Wavelength (nm)'], df['Reflectance Factor (%)']))
            reference_reflectances = []
            for wavelength in target_wavelengths_nm:
                if int(wavelength) in lookup:
                    reference_reflectances.append(lookup[int(wavelength)])
                else:
                    print(f'Warning, could not find match for {wavelength}')
            # convert to numpy and divide by 100 to get the decimal percentages
            reference_reflectances = np.array(reference_reflectances) / 100
            print(f'Reference reflectance curve parsed: {reference_reflectances}')
            return reference_reflectances
        else:
            return np.full(len(target_wavelengths_nm), 0.95, dtype=float)


    def parse_calibration_bands(self, calibration_xml):
        root = ET.parse(calibration_xml).getroot()
        zone = root.find(".//filter_zone")
        mosaic_size = int(zone.find("pattern_width").text)

        corr = root.find(f".//correction_matrix[name='{self.MATRIX_TYPE}']")

        if corr is not None and self.USE_CORRECTION_MATRIX:
            virtual_bands = []
            for vb in corr.findall("virtual_bands/virtual_band"):
                wavelength_nm = float(vb.find("wavelength_nm").text)
                coefficients = np.fromstring(
                    vb.find("coefficients").attrib["values"], sep=" ", dtype=np.float32
                )
                virtual_bands.append((wavelength_nm, coefficients))
            virtual_bands.sort(key=lambda item: item[0])
            wavelengths_nm = [item[0] for item in virtual_bands]
            correction_matrix = np.stack([item[1] for item in virtual_bands])
            return mosaic_size, None, wavelengths_nm, correction_matrix


        bands = []
        for band in zone.findall("bands/band"):
            if band.get("selected") != "true":
                continue
            mosaic_index = int(band.get("index"))
            wavelength_nm = float(band.find(".//wavelength_nm").text)
            bands.append((mosaic_index, wavelength_nm))
        bands.sort(key=lambda item: item[1])
        mosaic_indices = [item[0] for item in bands]
        wavelengths_nm = [item[1] for item in bands]
        return mosaic_size, mosaic_indices, wavelengths_nm, None


    def demosaic_to_cube(self, frame, mosaic_size, mosaic_indices=None, correction_matrix=None):
        if correction_matrix is not None:
            return self.apply_spectral_correction(
                self.demosaic_all_bands(frame, mosaic_size), correction_matrix
            )
        if mosaic_indices is None:
            raise ValueError("mosaic_indices required when no correction matrix is available")

        height, width = frame.shape
        out_h, out_w = height // mosaic_size, width // mosaic_size
        cube = np.empty((out_h, out_w, len(mosaic_indices)), dtype=np.float32)
        for band_i, mosaic_index in enumerate(mosaic_indices):
            dx = mosaic_index % mosaic_size
            dy = mosaic_index // mosaic_size
            cube[:, :, band_i] = frame[dy::mosaic_size, dx::mosaic_size][:out_h, :out_w]
        return cube

    
    def demosaic_all_bands(self, frame, mosaic_size):
        height, width = frame.shape
        n_bands = mosaic_size * mosaic_size
        out_h, out_w = height // mosaic_size, width // mosaic_size
        cube = np.empty((out_h, out_w, n_bands), dtype=np.float32)
        for mosaic_index in range(n_bands):
            dx = mosaic_index % mosaic_size
            dy = mosaic_index // mosaic_size
            cube[:, :, mosaic_index] = frame[dy::mosaic_size, dx::mosaic_size][:out_h, :out_w]
        return cube

    def apply_spectral_correction(self, cube, correction_matrix):
        # cube: (lines, samples, n_mosaic), matrix: (n_virtual, n_mosaic)
        # return cube @ correction_matrix # try this? 
        return cube @ correction_matrix.T
