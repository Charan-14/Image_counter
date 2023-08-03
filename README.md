# Image Counter Project for Korechi Innovations
The Repo consists of:
- image_pubisher.py - publishes webcam img as a rosmsg on /cam_img topic
- image_counter.py - saves images at a prime number counter and resets the counter if msg publsihed on /reset topic 
- image_path_saver.py - saves the img path names in a text file and resets if received msg on the /reset topic
- image_counter.launch - roslaunch file to launch the three nodes together
- image_counting_demo.mkv - demo video to showcase the working of the project

To reset the counter the following msg is published through the command line interface
`rostopic pub /reset std_msgs/Empty "{}"`
