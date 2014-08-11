How to use the ALPS doxygen template:

1. Copy content of thid directory to the directory containing the .doxyfile of your project.

2. Add or edit following settings in your .doxyfile:

HTML_STYLESHEET = "doxygen_ALPS.css"

HTML_EXTRA_FILES = "doxy_src\tabs.css"
HTML_EXTRA_FILES += "doxy_src\alps_docubar.png"
HTML_EXTRA_FILES += "doxy_src\alps_navmenubar.png"
HTML_EXTRA_FILES += "doxy_src\alps_navmenutail.png"
HTML_EXTRA_FILES += "doxy_src\alps_hoverlink-stripe.png"
HTML_EXTRA_FILES += "doxy_src\alps_currentlink-stripe.png"
HTML_EXTRA_FILES += "doxy_src\alps_topline_wide.png"
HTML_EXTRA_FILES += "doxy_src\alps_logo.gif"

3. Generate your doxygen and go home!

2011, Dusan Jurica, ALPS Electric Czech