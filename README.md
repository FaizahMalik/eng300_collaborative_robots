
### Info

This work is part of my Mechanical Engineering Bachelor's Dissertation project.
The robots and software were inherited from a previous project, and the system capabilities were improved upon as part of the current work.

Updates comapred to previous software from ENGR407:

 - The robots now use DigiMesh networking to communicate with each other using on board Digi Xbee 3 devices.
   - The robots previously relied upon a computer running a single ROS core within the same WiFi network for communication.
 - The system has been made more modular and scalable using Gonçalo Martins' MRGS framework, through the implementation of a Data Interface node which handles a lot of the data transfer between topics & compresses the maps
 - The updated map merging algorithm (Stefano Carpin's Hough Transform) uses less CPU and memory resources when compared with the previously implemented algorithm (Jiri Horner's Multirobot Map Merge), this was experimentally validated using bag files.


---

### Branches


Both of the below branches have the new Xbee communication and MRGS data handling capabilities, just with different map merging algorithms integrated.

- **map_merge_testing_mexplore** - Full software with the previously implemented map merging algorithm (a modified version of Jiri Horner's Multirobot Map Merge)

- **map_merge_testing_mrgs** - Full software with the map merging algorithm used in the MRGS framework (MRGS framework is from  Gonçalo Martins, and the map merging is from Stefano Carpin using the Hough Transform method)







