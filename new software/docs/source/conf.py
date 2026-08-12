from pathlib import Path

DOCS_DIR = Path(__file__).resolve().parent.parent
DOXYGEN_XML = DOCS_DIR / "doxygen" / "xml"

project = "AnkleDocs"
copyright = "2026, Oscar Tesniere"
author = "Oscar Tesniere"
release = "v1.0"

extensions = [
    "myst_parser",
    "breathe",
]

breathe_projects = {
    "framework": str(DOXYGEN_XML),
}

breathe_default_project = "framework"

templates_path = ["_templates"]
exclude_patterns = []

html_theme = "alabaster"
html_static_path = ["_static"]
highlight_language = "cpp"