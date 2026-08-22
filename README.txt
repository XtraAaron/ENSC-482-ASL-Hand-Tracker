====================================================================
ASL Hand Tracker: Files In This Folder
====================================================================

Source code:
    handTracker.py
        Captures webcam video,
        runs MediaPipe's HandLandmarker model to extract 21 hand landmarks per frame,
        applies OneEuroFilter smoothing to reduce jitter,
        and sends the filtered landmark data over UDP to Blender.

    handEditFile.py
        Runs inside Blender. 
        Listens for landmark data from handTracker.py, 
        rotates the armature's finger bones and wrist to match the tracked hand pose,
        and forwards the resulting bone rotation values over UDP to aslReader.exe.

    aslReader.cpp
        C++ program that listens for bone rotation data,
        runs a decision tree to classify the pose into an ASL letter or number, 
        and prints result to the console.

    aslReader.exe
        Precompiled Windows executable of aslReader.cpp. 
        run_all.bat recompiles this from source each time it's run.

  run_all.bat
      Windows batch script that launches handTracker.py (using the "aslenv" Python virtual environment) 
      and aslReader.exe together in separate console windows. 
      Does not launch Blender, that must be started manually first (described in Report Appendix).

Subfolders:
    source/
        PuppetHand.blend
            The rigged robot/puppet hand model and armature ("Armature"
            object, with a "Palm" bone for wrist rotation) that
            handEditFile.py drives.

        PuppetHand.blend1
            Blender auto-save backup of the above.

  textures/
        HandStand_n.png,
        Handstand_a.png,
        Segment_n.png,
        Top_n.png, and
        Wood.png 
            Textures used to shade the PuppetHand model.

Supporting files:
    hand_landmarker.task
        MediaPipe's pretrained hand landmark detection model file,
        required by handTracker.py.

    notesForLetters.txt and 
    note.txt
        Notes on how wrist rotation and finger curl ranges map to each ASL letter/number, 
        used to calibrate the decision tree in aslReader.cpp.

    asl-chart-source.jpg and 
    asl-number-reference.jpg
        Reference images of the ASL alphabet and number handshapes used during development.

    mediaPipeDiagram.jpg
        Reference diagram of MediaPipe's 21 hand landmark points and their index numbers.

    aslReaderTree1.jpg,
    aslReaderTree2.jpg, and
    aslReaderTree2_bitMoreContext.jpg
        Diagrams of the decision tree logic implemented in aslReader.cpp.
