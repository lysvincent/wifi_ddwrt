wifi_ddwrt
==========

General description
--------------------
This package implements WiFi fingerprinting algorithm for robot rough-localization. 

Usage and Workflow
-------------------
There are two launch files present in the package, namely ddwrt_mapping and ddwrt_localizing. Before the localizing part of the package can be utilised, ddwrt_mapping is required to be implemented at least once, in order to provide a fingerprint-map for the use of localization.

The ddwrt_mapping can be ros-launched regardless of the position of the user in the directories, however the ros-launching of ddwrt_localization requires the user to be in the directory,where the fingerprint-map is stored,which by default is at the ros hidden file(.ros/). 

The .yaml file,which is created by a service in the fingerprinting node, that contains details regarding the detected access points, is also saved in the ros hidden file(.ros/). 

Conclusion.
**Both wifi_ddwrt node is needed in both the mapping and localization process.**
**ddwrt_fingerprinting needs to be launched where the fingerprint-map is stored**

Node: wifi_ddwrt
--------------------

The node that parses the surrounding access points that are detected by the subject's router.
#### Parameters
**hostname** *(string)*
Hostname of the robot's router

**username** *(string)*
Username of the robot'S router

**password** *(string)*
Password to access the router's ddwrt programme

**macaddr** *(string)*
Mac-addresses of the specified access points

#### Published Topics
**ddwrt/sitesurvey** *(wifi_ddwrt_msg::SiteSurvey)*
Publishes a list of access points' details detected by the subject currently.

**ddwrt/accesspoint** *(wifi_ddwrt_msg::AccessPoint)*
Publishes the details of the access point that the router is currently connected to.

**ddwrt/seen_specified_aps** *(wifi_ddwrt_msg::CellAp)*
Publishes a list of specified access points' details including the current signal strength. 

Node: approximate
--------------------

The node that provides an estimated pose and posewithcovariance based on the current detected access points and the recorded fingerprints.

#### Parameters
**database** *(string)*
The name of the file where the fingerprints are stored.

**percentage_taken**
Amount of positions whose euclidean distance are the least (closest to zero)

**macaddr** *(string)*
Mac-addresses of the specified access points.

#### Published Topics
**ddwrt/pose** *(geometry_msgs::PoseStamped)*
publishes the current estimated pose.

**ddwrt/posewithcovariance** *(geometry_msgs::PoseWithCovarianceStamped)*
publishes the current localization state.

#### Subscribed Topics
**ddwrt/seen_specified_aps** *(wifi_ddwrt_msgs::CellAp)*
Acquires a list of specified access points' details including the current signal strength.

#### Services
**setIniPose** *(wifi_ddwrt_msgs::SetIniPose)*
Sets the initial pose (x-/y-Position) for the robot.


Node: fingerprinting
--------------------

The node that saves fingerprints which consists of access points versus base_link tf.

#### Parameters
**c_width** *(float)*
The width of a cell.

**c_height** *(float)*
The height of a cell.

**macaddr** *(string)*
Mac-addresses of the specified access points.

**essid** *(string)*
Essid of the specified access points.

**database** *(string)*
The name of the file where the fingerprints should be stored.

**list_survey** *(string)*
The name of the file where the details' of the seen surrounding access points should be stored.

#### Subscribed Topics
**ddwrt/seen_specified_aps** *(wifi_ddwrt_msgs::CellAp)*
Acquires a list of specified access points' details including the current signal strength.

**ddwrt/sitesurvey** *(wifi_ddwrt_msgs::SiteSurvey)*
Acquires a list of access points' details detected by the subject currently.

#### Services
**save_list_of_surveyed_aps** *(wifi_ddwrt_msg::SaveList)*
save a list of surrounding access points in .yaml file.
