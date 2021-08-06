#!/usr/bin/env python3

import rospy

from sss_pytools.srv import UnionMeshes, UnionMeshesResponse

from tool_calibration.srv import CalibrationUrdfUpdate, \
                                       CalibrationUrdfUpdateRequest, \
                                       CalibrationUrdfUpdateResponse

from tool_calibration.srv import CalibrationUrdfRetrieve, \
                                       CalibrationUrdfRetrieveRequest, \
                                       CalibrationUrdfRetrieveResponse


from xml.dom import minidom
import yaml

class CalibrationUrdfServer:
    def __init__(self):
        self.calibration_update_urdf_service = rospy.Service(
            'calibration_urdf_server/calibration_urdf_update', CalibrationUrdfUpdate, self.update_urdf_cbf)

        self.calibration_urdf_retrieve_service = rospy.Service(
            'calibration_urdf_server/calibration_urdf_retrieve', CalibrationUrdfRetrieve, self.urdf_retrieve_cbf)

        # only supports 'xacro:property' elements at the moment
        self.element_tag_name = 'xacro:property'

        # TODO: replace hardcoded directory with better way to load the yaml. Roslaunch param's maybe?
        self.system_catkin_package_dir = "/home/ir4/noetic/src/"

        # dict containing the calibration tool surface linked to the file location name and dict of urdf element names to req value
        stream = open(self.system_catkin_package_dir + 'ir4_tool_calibration/tool_calibration/config/urdf_info.yaml', 'r')
        self.urdf_info_dict = yaml.load(stream, Loader=yaml.FullLoader)

    def update_urdf_cbf(self, req):
        rospy.loginfo('Calibration urdf update callback')

        response = CalibrationUrdfUpdateResponse()

        success, urdf_file_location, element_tag_name_to_calibration_value_dict, parsed_dom = self.load_urdf(req.robot_tool_surface)
        if not success:
            response.success = success
            return response

        property_elements = parsed_dom.getElementsByTagName(
            self.element_tag_name)

        for element in property_elements:
            node_name = element.getAttribute('name')
            if node_name in element_tag_name_to_calibration_value_dict.keys():
                new_value = format(getattr(req, element_tag_name_to_calibration_value_dict[node_name]), 'f')
                rospy.loginfo('Changing ' + node_name + ' to ' + new_value)

                element.setAttribute('value', new_value)

        rospy.loginfo("Saving as: ")
        rospy.loginfo("\n" + parsed_dom.toxml())

        output_file = open(
            urdf_file_location, "w")
        output_file.write(parsed_dom.toxml())
        output_file.close()
    
        response.success = success
        return response

    def urdf_retrieve_cbf(self, req):
        response = CalibrationUrdfRetrieveResponse()

        success, urdf_file_location, element_tag_name_to_calibration_value_dict, parsed_dom = self.load_urdf(req.robot_tool_surface)
        if not success:
            response.success = success
            return response

        property_elements = parsed_dom.getElementsByTagName(
            self.element_tag_name)

        for element in property_elements:
            node_name = element.getAttribute('name')
            if node_name in element_tag_name_to_calibration_value_dict.keys():
                # set the response value based on the urdf
                setattr(response, element_tag_name_to_calibration_value_dict[node_name], float(element.getAttribute('value')))

        response.success = True
        return response

    def load_urdf(self, robot_tool_surface):
        if robot_tool_surface not in self.urdf_info_dict.keys():
            rospy.logerr("Error. Do not have a urdf file name for " +
                         robot_tool_surface)
            return (False, "", "", "")

        # set variables from dict based on tool surface (for readability)
        urdf_file_location = self.system_catkin_package_dir + self.urdf_info_dict[robot_tool_surface]["file_location"]
        element_tag_name_to_calibration_value_dict = self.urdf_info_dict[robot_tool_surface]["calibration_value_names"]

        try:
            parsed_dom = minidom.parse(urdf_file_location)
        except:
            # failed to parse file
            rospy.logerr('Failed to parse file at ' + urdf_file_location)
            return (False, "", "", "")
        
        return (True, urdf_file_location, element_tag_name_to_calibration_value_dict, parsed_dom)

def main():
    rospy.init_node('calibration_urdf_server')
    cus = CalibrationUrdfServer()
    rospy.loginfo('CalibrationUrdfUpdateServer has started')
    rospy.spin()


if __name__ == "__main__":
    main()
