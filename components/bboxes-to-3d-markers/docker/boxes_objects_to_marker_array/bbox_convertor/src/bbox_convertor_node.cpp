/*
 * bbox_convertor_node.cpp
 *
 *  Created on: May 13, 2019
 *      Author: Igor Makhtes <igor@cogniteam.com>
 *
 *
 * Cogniteam LTD CONFIDENTIAL
 *
 * Unpublished Copyright (c) 2016-2017 Cogniteam,        All Rights Reserved.
 *
 * NOTICE:  All information contained  herein  is,  and  remains the property
 * of Cogniteam.   The   intellectual   and   technical   concepts  contained
 * herein are proprietary to Cogniteam and may  be  covered  by  Israeli  and
 * Foreign Patents, patents in process,  and  are  protected  by trade secret
 * or copyright law. Dissemination of  this  information  or  reproduction of
 * this material is strictly forbidden unless  prior  written  permission  is
 * obtained  from  Cogniteam.  Access  to  the  source  code contained herein
 * is hereby   forbidden   to   anyone  except  current  Cogniteam employees,
 * managers   or   contractors   who   have   executed   Confidentiality  and
 * Non-disclosure    agreements    explicitly    covering     such     access
 *
 * The copyright notice  above  does  not  evidence  any  actual  or intended
 * publication  or  disclosure    of    this  source  code,   which  includes
 * information that is confidential  and/or  proprietary,  and  is  a   trade
 * secret, of   Cogniteam.    ANY REPRODUCTION,  MODIFICATION,  DISTRIBUTION,
 * PUBLIC   PERFORMANCE,  OR  PUBLIC  DISPLAY  OF  OR  THROUGH USE   OF  THIS
 * SOURCE  CODE   WITHOUT   THE  EXPRESS  WRITTEN  CONSENT  OF  Cogniteam  IS
 * STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE LAWS AND INTERNATIONAL
 * TREATIES.  THE RECEIPT OR POSSESSION OF  THIS SOURCE  CODE  AND/OR RELATED
 * INFORMATION DOES  NOT CONVEY OR IMPLY ANY RIGHTS  TO  REPRODUCE,  DISCLOSE
 * OR  DISTRIBUTE ITS CONTENTS, OR TO  MANUFACTURE,  USE,  OR  SELL  ANYTHING
 * THAT      IT     MAY     DESCRIBE,     IN     WHOLE     OR     IN     PART
 *
 */

#include <opencv2/highgui/highgui.hpp>
#include <iostream> // std::cout
#include <string>   // std::string, std::to_string

#include <ros/ros.h>
#include <object_msgs/ObjectsInBoxes.h>

#include <map>

//for jetson-inference detection
#include <vision_msgs/Detection2DArray.h>

using namespace std;

class ObjectsMsgConvertor
{

public:
    ObjectsMsgConvertor()
    {

        detectedVisionMsgObjectsSubscriber_ = node_.subscribe("/vision_msg_detected_objects", 1,
                                                              &ObjectsMsgConvertor::detectedObjectsCallback, this);

        openVinoObjectsPub_ =
            node_.advertise<object_msgs::ObjectsInBoxes>("openvino_toolkit/detected_objects", 1);

        initClasses();    
    }

    ~ObjectsMsgConvertor() {}

public:
    void detectedObjectsCallback(const vision_msgs::Detection2DArray::Ptr &objects)
    {

        object_msgs::ObjectsInBoxes openVinoObjects;

        openVinoObjects.objects_vector.resize(objects->detections.size());

        for (int i = 0; i < objects->detections.size(); i++)
        {

            object_msgs::ObjectInBox objectInBox;
            objectInBox.roi.width = objects->detections[i].bbox.size_x;
            objectInBox.roi.height = objects->detections[i].bbox.size_y;
            objectInBox.object.object_name = convertIdToType(objects->detections[i].results[0].id);
            objectInBox.roi.y_offset =
                objects->detections[i].bbox.center.y - (objects->detections[i].bbox.size_y / 2);
            objectInBox.roi.x_offset =
                objects->detections[i].bbox.center.x - (objects->detections[i].bbox.size_x / 2);

            openVinoObjects.objects_vector[i] = objectInBox;
        }

        openVinoObjectsPub_.publish(openVinoObjects);
    }

    string convertIdToType(int id)
    {    
       
        map<int,string>::iterator it = objects_classes.find(id);
        if(it != objects_classes.end())
        {
            return objects_classes[id];
        }      

    }

    void initClasses(){

        objects_classes[1] = "person";
        objects_classes[2] = "bicycle";
        objects_classes[3] = "car";
        objects_classes[4] = "motorcycle";
        objects_classes[5] = "airplane";
        objects_classes[6] = "bus";
        objects_classes[7] = "train";
        objects_classes[8] = "truck";
        objects_classes[9] = "boat";
        objects_classes[10] = "traffic light";
        objects_classes[11] = "fire hydrant";
        objects_classes[12] = "street sign";
        objects_classes[13] = "stop sign";
        objects_classes[14] = "parking meter";
        objects_classes[15] = "bench";
        objects_classes[16] = "bird";
        objects_classes[17] = "cat";
        objects_classes[18] = "dog";
        objects_classes[19] = "horse";
        objects_classes[20] = "sheep";
        objects_classes[21] = "cow";
        objects_classes[22] = "elephant";
        objects_classes[23] = "bear";
        objects_classes[24] = "zebra";
        objects_classes[25] = "giraffe";
        objects_classes[26] = "hat";
        objects_classes[27] = "backpack";
        objects_classes[28] = "umbrella";
        objects_classes[29] = "shoe";
        objects_classes[30] = "eye glasses";
        objects_classes[31] = "handbag";
        objects_classes[32] = "tie";
        objects_classes[33] = "suitcase";
        objects_classes[34] = "frisbee";
        objects_classes[35] = "skis";
        objects_classes[36] = "snowboard";
        objects_classes[37] = "sports ball";
        objects_classes[38] = "kite";
        objects_classes[39] = "baseball bat";
        objects_classes[40] = "baseball glove";
        objects_classes[41] = "skateboard";
        objects_classes[42] = "surfboard";
        objects_classes[43] = "tennis racket";
        objects_classes[44] = "bottle";
        objects_classes[45] = "plate";
        objects_classes[46] = "wine glass";
        objects_classes[47] = "cup";
        objects_classes[48] = "fork";
        objects_classes[49] = "knife";
        objects_classes[50] = "spoon";
        objects_classes[51] = "bowl";
        objects_classes[52] = "banana";
        objects_classes[53] = "apple";
        objects_classes[54] = "sandwich";
        objects_classes[55] = "orange";
        objects_classes[56] = "broccoli";
        objects_classes[57] = "carrot";
        objects_classes[58] = "hot dog";
        objects_classes[59] = "pizza";
        objects_classes[60] = "donut";
        objects_classes[61] = "cake";
        objects_classes[62] = "chair";
        objects_classes[63] = "couch";
        objects_classes[64] = "potted plant";
        objects_classes[65] = "bed";
        objects_classes[66] = "mirror";
        objects_classes[67] = "dining table";
        objects_classes[68] = "window";
        objects_classes[69] = "desk";
        objects_classes[70] = "toilet";
        objects_classes[71] = "door";
        objects_classes[72] = "tv";
        objects_classes[73] = "laptop";
        objects_classes[74] = "mouse";
        objects_classes[75] = "remote";
        objects_classes[76] = "keyboard";
        objects_classes[77] = "cell phone";
        objects_classes[78] = "microwave";
        objects_classes[79] = "oven";
        objects_classes[80] = "toaster";
        objects_classes[81] = "sink";
        objects_classes[82] = "refrigerator";
        objects_classes[83] = "blender";
        objects_classes[84] = "book";
        objects_classes[85] = "clock";
        objects_classes[86] = "vase";
        objects_classes[87] = "scissors";
        objects_classes[88] = "teddy bear";
        objects_classes[89] = "hair drier";
        objects_classes[90] = "toothbrush";
        objects_classes[91] = "hair brush";
    }

private:
    ros::NodeHandle node_;

    ros::Subscriber detectedVisionMsgObjectsSubscriber_;

    ros::Publisher openVinoObjectsPub_;

    std::map<int, string> objects_classes;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bbox_convertor_node");

    ObjectsMsgConvertor objectsMsgConvertor;

    ros::spin();
    return 0;
}

