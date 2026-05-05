import os
import catkin_pkg.package

catkin_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
catkin_package = catkin_pkg.package.parse_package(
    os.path.join(catkin_dir, catkin_pkg.package.PACKAGE_MANIFEST_FILENAME)
)

project = catkin_package.name
copyright = '2026, Universal Robots A/S'
author = 'Sergi Romero'

version = catkin_package.version
release = catkin_package.version

extensions = [
    'sphinx_tabs.tabs',
    'sphinx.ext.todo',
    'sphinx_copybutton',
    'sphinx_rtd_theme',
]

copybutton_exclude = '.linenos, .gp, .go'
todo_include_todos = True
todo_emit_warnings = True

master_doc = 'index'
language = 'en'
exclude_patterns = []
pygments_style = None

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
html_logo = "_static/images/ur_logo.svg"
html_favicon = "_static/images/icon.svg"
html_css_files = ['css/ur_theme.css']

html_theme_options = {
    'logo_only': False,
    'globaltoc_collapse': True,
    'globaltoc_maxdepth': None,
}
html_show_sourcelink = False
html_show_sphinx = False

htmlhelp_basename = f'{project}doc'

latex_elements = {}
latex_documents = [
    (
        master_doc,
        f'{project}.tex',
        f'{project} Documentation',
        author,
        'manual',
    ),
]

man_pages = [
    (master_doc, project, f'{project} Documentation', [author], 1),
]

texinfo_documents = [
    (
        master_doc,
        project,
        f'{project} Documentation',
        author,
        project,
        'One line description of project.',
        'Miscellaneous',
    ),
]

epub_title = project
epub_exclude_files = ['search.html']
