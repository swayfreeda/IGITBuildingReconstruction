######################################################################
# Automatically generated by qmake (2.01a) Wed Nov 5 11:17:24 2014
######################################################################
QT += qt3support
QMAKE_CXXFLAGS += -frounding-math

# Input
HEADERS += \
    sw_dataType.h \
    sw_glviewer.h \
    sw_mainwindow.h \
    sw_dataIO.h \
    sw_functions.h \
    sw_floorplan.h \
    unionFindGraph.h \
    sw_codingEdit.h \
    sw_graph_str.h \
    graph.h \
    energy.h \
    block.h \
    sw_alpha_expansion.h
FORMS += mainwindow.ui \
    FloorPlan.ui
SOURCES += main.cpp \
    sw_glviewer.cpp \
    sw_dataType.cpp \
    sw_mainwindow.cpp \
    sw_dataIO.cpp \
    sw_functions.cpp \
    sw_floorplan.cpp \
    unionFindGraph.cpp \
    sw_graph_str.cpp \
    sw_codingEdit.cpp \
    graph.cpp \
    alpha_expansion.cpp \
    maxflow.cpp

INCLUDEPATH +=  /usr/include/qt4/QtCore     \
                 /usr/include/qt4/QtGui      \
                 /usr/include/qt4/QtOpenGL    \
                 /usr/include/qt4             \
                 /usr/include/qt3/          \
                 /usr/include/qt4/QtXml/

LIBS  +=    -L/usr/X11R6/lib64 -lQtOpenGL -lQtGui -lQtCore  -lpthread\
             -lQGLViewer -lGLEW -lglut  -lGL  -lGLU  \
            -L/usr/local/lib/   -lopencv_core  -lopencv_flann -lopencv_highgui -lopencv_imgproc -lopencv_legacy

