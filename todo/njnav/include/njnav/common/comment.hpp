/** \mainpage
\section intro Introduction

NJNAV (NanJiang NAVigation) is a cross-platfrom C++ library for autonomous mobile robots. 
The NJNAV contains numerous algorithms for robot auto navigation including 
SLAM(Simutanuous Localization And Mapping), localization and navigation. 
It provides not only simple used packages for easily/quickly building your mobile robot projects, 
but also detailed API for customizing your own algorithms on specific application scenarios.

NJNAV is cross-platfrom and has been successfully compiled and deployed on Linux(Ubuntu) and Windows.

We are sorry that NJNAV is not open-source yet.
	
\section prerequisites Prerequisites
	1. Boost
	2. Eigen
	3. Glib(>=2.0)
	4. log4cpp
	5. OpenCV

\section build How to Build

	Linux:
		cd 'source directory'
		mkdir build && cd build
		cmake ..
		make 
		sudo make install
	Windows:
		open 'Cmake GUI'
		choose your complier
		build
		compile with your complier

\author symao(maoshuyuan@njrobot.com)

*/