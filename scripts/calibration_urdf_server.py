#!/usr/bin/env python3

import rospy

from sss_pytools.srv import UnionMeshes, UnionMeshesResponse

from tool_point_calibration.srv import CalibrationUrdfUpdate, \
                                       CalibrationUrdfUpdateRequest, \
                                       CalibrationUrdfUpdateResponse

from tool_point_calibration.srv import CalibrationUrdfRetrieve, \
                                       CalibrationUrdfRetrieveRequest, \
                                       CalibrationUrdfRetrieveResponse


from xml.dom import minidom

class CalibrationUrdfServer:
    def __init__(self):
        self.calibration_update_urdf_service = rospy.Service(
            'calibration_urdf_server/calibration_urdf_update', CalibrationUrdfUpdate, self.update_urdf_cbf)

        self.calibration_urdf_retrieve_service = rospy.Service(
            'calibration_urdf_server/calibration_urdf_retrieve', CalibrationUrdfRetrieve, self.urdf_retrieve_cbf)

        # only supports 'xacro:property' elements at the moment
        self.element_tag_name = 'xacro:property'

        self.system_catkin_package_dir = "/home/ir4/noetic/src/"

        # dict containing the calibration tool surface linked to the file location name and dict of urdf element names to req value
        self.urdf_info_dict = {
            "kr8_r1420_rcb/welder_tool_surface": {"file_location":"kuka/kuka_manipulators/kuka_common_support/urdf/kr8_calibration_values.urdf.xacro", "calibration_value_names":{"welder_tip_x": "calibration_x", "welder_tip_y": "calibration_y", "welder_tip_z": "calibration_z"}}
        }

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
                new_value = str(getattr(req, element_tag_name_to_calibration_value_dict[node_name]))
                rospy.loginfo('Changing ' + node_name + ' to ' + new_value)

                element.setAttribute('value', new_value)

        rospy.loginfo("Saving as: ")
        rospy.loginfo(parsed_dom.toxml())

        output_file = open(
            urdf_file_location, "w")
        output_file.write(parsed_dom.toxml())
        output_file.close()

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
    cuus = CalibrationUrdfServer()
    rospy.loginfo('CalibrationUrdfUpdateServer has started')
    rospy.spin()


if __name__ == "__main__":
    main()
