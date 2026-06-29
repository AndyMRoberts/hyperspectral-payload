import json
from pathlib import Path
from typing import Any

from foxglove.layouts import (
    Layout,
    MarkdownConfig,
    MarkdownPanel,
    TabContainer,
    TabItem,
)

FOXGLOVE_DIR = Path(__file__).parent
OUTPUT_PATH = FOXGLOVE_DIR / "my_layout.json"

CUSTOM_TAB_PANEL_ID = "Tab!hyper_custom"
CUSTOM_INNER_TAB_ID = "Tab!hyper_custom_inner"
CUSTOM_MARKDOWN_PANELS = (
    ("Markdown!hyper_custom_md1", "Tab 1", "Hello, world!"),
    ("Markdown!hyper_custom_md2", "Tab 2", "Hello, world!"),
)

HYPER_LAYOUTS = (
    ("HSI Images", FOXGLOVE_DIR / "hyper_hsi_images.json"),
    ("Other Sensors", FOXGLOVE_DIR / "hyper_other_sensors.json"),
)


def build_custom_inner_tab_config() -> dict[str, Any]:
    layout = Layout(
        content=TabContainer(
            selected_tab_index=1,
            tabs=[
                TabItem(
                    title=title,
                    content=MarkdownPanel(
                        config=MarkdownConfig(markdown=markdown),
                    ),
                )
                for _, title, markdown in CUSTOM_MARKDOWN_PANELS
            ],
        ),
    )
    return json.loads(layout.to_json())["content"]


def load_opaque_layout(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text())


def merge_dicts(target: dict[str, Any], source: dict[str, Any]) -> None:
    for key, value in source.items():
        if key in target and target[key] != value:
            raise ValueError(f"Conflicting values for {key!r} while merging layouts")
        target[key] = value


def build_merged_layout() -> dict[str, Any]:
    custom_inner = build_custom_inner_tab_config()
    hyper_layouts = [
        (title, load_opaque_layout(path)) for title, path in HYPER_LAYOUTS
    ]

    merged: dict[str, Any] = {
        "configById": {},
        "globalVariables": {},
        "userNodes": {},
        "playbackConfig": {"speed": 1},
    }

    for panel_id, _, markdown in CUSTOM_MARKDOWN_PANELS:
        merged["configById"][panel_id] = {
            "renderMode": "static",
            "markdown": markdown,
        }

    merged["configById"][CUSTOM_INNER_TAB_ID] = {
        "activeTabIdx": custom_inner["selectedTabIndex"],
        "tabs": [
            {
                "title": tab["title"],
                "layout": panel_id,
            }
            for tab, (panel_id, _, _) in zip(
                custom_inner["tabs"], CUSTOM_MARKDOWN_PANELS, strict=True
            )
        ],
    }

    for _, layout_data in hyper_layouts:
        merge_dicts(merged["configById"], layout_data.get("configById", {}))
        merge_dicts(merged["globalVariables"], layout_data.get("globalVariables", {}))
        merge_dicts(merged["userNodes"], layout_data.get("userNodes", {}))

    merged["configById"][CUSTOM_TAB_PANEL_ID] = {
        "activeTabIdx": 0,
        "tabs": [
            {
                "title": "Overview",
                "layout": CUSTOM_INNER_TAB_ID,
            },
            *[
                {
                    "title": title,
                    "layout": layout_data["layout"],
                }
                for title, layout_data in hyper_layouts
            ],
        ],
    }

    merged["layout"] = CUSTOM_TAB_PANEL_ID

    if "drawerConfig" in hyper_layouts[0][1]:
        merged["drawerConfig"] = hyper_layouts[0][1]["drawerConfig"]

    return merged


def main() -> None:
    output_path = OUTPUT_PATH
    output_path.write_text(json.dumps(build_merged_layout(), indent=2))
    print(f"Wrote merged layout to {output_path}")


if __name__ == "__main__":
    main()
