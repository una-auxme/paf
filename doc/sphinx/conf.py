# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = "PAF"
copyright = "2025, PAF"
author = "PAF"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.viewcode",
    "sphinx_autodoc_typehints",
    "myst_parser",
]

source_suffix = {
    ".rst": "restructuredtext",
    ".txt": "markdown",
    ".md": "markdown",
}

templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

language = "en"


def skip(app, what, name, obj, would_skip, options):
    if name in ("__init__",):
        return False
    return would_skip


def setup(app):
    app.connect("autodoc-skip-member", skip)


# autodoc config:
autodoc_class_signature = "mixed"
autodoc_member_order = "bysource"
autodoc_preserve_defaults = True

# markdown config
markdown_bullet = "-"
markdown_anchor_signatures = True
