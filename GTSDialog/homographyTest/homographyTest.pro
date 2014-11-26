SOURCES += \
    main.cpp


INCLUDEPATH += /usr/share/qt4/mkspecs/linux-g++-64   \
                 /usr/include/qt4/QtCore     \
                 /usr/include/qt4/QtGui      \
                 /usr/include/qt4/QtOpenGL    \
                 /usr/include/qt4             \
                 /usr/include/qt3/          \
                 /usr/include/qt4/QtXml/    \
                 /usr/include/QGLViewer      \
                 # ------opencv-----------
                 /usr/local/include             \
                 /usr/local/include/opencv      \
                 /usr/local/include/opencv2
                 # -------FLANN------------



LIBS +=    -L/usr/local/lib/ -lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d \
         -L/usr/local/lib/ -lopencv_flann -lopencv_highgui -lopencv_imgproc -lopencv_legacy
