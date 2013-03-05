Source Code for Group 9
=======

This is only the Java source code for our project - it uses javacv and v4l4j which are fairly tricky to setup so I'm also providing a zip file with all the dependencies bundled up [here](http://dl.dropbox.com/u/4739498/sdp2012-group9.zip), you only need to create an new LeJOS Eclipse project, import the source code and use "Project Properties for PC -> Java Build Path -> Libraries" to add the following libraries (paths relative to the provided package):

* javacv/javacpp.jar
  * Native library location: opencv/lib
* javacv/javacv-linux-x86_64.jar
  * Native library location: opencv/lib
* javacv/javacv-linux-x86.jar
  * Native library location: opencv/lib
* javacv/javacv.jar
  * Native library location: opencv/lib
* v4l4j/v4l4j.jar
  * Native library location: v4l4j
* jbox2d/jbox2d-library-2.1.2.1-SNAPSHOT-jar-with-dependencies.jar
* jna/jna.jar

If you do this, it should run with the following run configuration:

Main Class: sdp.server.PcServer
Environment:
  * LD_LIBRARY_PATH=${workspace_loc}/../v4l4j:${workspace_loc}/../bluez/lib:
