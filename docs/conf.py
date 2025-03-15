# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = "HomeR"
copyright = "2025, linzhanguca"
author = "linzhanguca"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = []

templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]
html_logo = "images/homer_logo.png"
html_theme_options = {"logo_only": False}
extensions = [
    # "sphinx.ext.autodoc",
    # "sphinx.ext.napoleon",
    "sphinx.ext.githubpages",
    "sphinx_copybutton",
]
# Strip input prompts (e.g., "$ ", ">>> ") from copied text
copybutton_exclude = '.linenos, .gp, .go'
