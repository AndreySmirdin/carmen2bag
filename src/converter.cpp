#include <converter.h>


Converter::Converter(char *in, char *out, char *params) {
    ifs = std::ifstream(in);
    if (!ifs)
        throw std::logic_error("File doesn`t exist.");

    bag.open(out, rosbag::bagmode::Write);
    ofs = std::ofstream(params);

    stamp = ros::Time::now();

    ros::param::param<double>("~global_rate", rate, 40);

    ros::param::param<std::string>("~robot_link", robot_link, "base_link");
    ros::param::param<std::string>("~odom_link", odom_link, "odom");
    ros::param::param<std::string>("~odom_robot_link", odom_robot_link, "odom_robot_link");

    ros::param::param<std::string>("~ROBOTLASER1_link", ROBOTLASER1_link, "ROBOTLASER1_link");


    links = {{"ROBOT",       robot_link},
             {"ODOM",        odom_link},
             {"ROBOTODOM",   odom_robot_link},
             {"ROBOTLASER1", ROBOTLASER1_link}};
}


Converter::~Converter() {
    bag.close();
}


void Converter::convert() {
    std::string data;

    std::cout << "Convartation has started.\n";
    int cnt = 0;

    while (ifs.good()) {
        ifs >> data; // Reading next topic.

        if (data == "#")
            getline(ifs, data); // Skip line.

        if (data == "PARAM") {
            getline(ifs, data);
            ofs << "PARAM" << data << "\n";
        }

        if (data == "RAWLASER1") {
            handleLaserMsg();

            bag.write("/RAWLASER1", laser_msg.header.stamp, laser_msg);
            laser_msg.header.seq++;

        }

        else if (data == "ROBOTLASER1") {
            handleRobotLaserMsg();

            bag.write("/ROBOTLASER1", laser_msg.header.stamp, laser_msg);
            laser_msg.header.seq++;

        }

        else if (data == "ODOM") {
            handleOdom();

            bag.write("/ODOM", pose_msg.header.stamp, pose_msg);
            bag.write("/tf", pose_msg.header.stamp, tf2_msg);

            pose_msg.header.seq++;
            tf_odom_robot_msg.header.seq++;

        }

        else
            continue;

        incrementStamp();
        tf2_msg = tf::tfMessage();

        std::cout << "Line " << ++cnt << " was successfully converted.\n";
    }

    std::cout << "Convertation has been finished!";
}


void Converter::handleOdom() {
    double x, y, th, d4, d5, d6, curTime; // d4, d5, d6 are fields we don`t need.
    ifs >> x >> y >> th >> d4 >> d5 >> d6 >> curTime;

    pose_msg.header.stamp = ros::Time(curTime);

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    pose_msg.pose.pose.position.x = x;
    pose_msg.pose.pose.position.y = y;
    pose_msg.pose.pose.position.z = 0.0;
    pose_msg.pose.pose.orientation = odom_quat;

    odom_link = links["ODOM"];
    robot_link = links["ROBOT"];

    pose_msg.header.frame_id = odom_link;
    pose_msg.child_frame_id = robot_link;


    tf_odom_robot_msg.header.stamp = pose_msg.header.stamp;

    tf_odom_robot_msg.header.frame_id = odom_link;
    tf_odom_robot_msg.child_frame_id = robot_link;

    tf_odom_robot_msg.transform.translation.x = x;
    tf_odom_robot_msg.transform.translation.y = y;
    tf_odom_robot_msg.transform.translation.z = 0.0;
    tf_odom_robot_msg.transform.rotation = odom_quat;

    tf2_msg.transforms.push_back(tf_odom_robot_msg);
}


void Converter::handleLaserMsg() {
    laser_msg.header.frame_id = "base_link";
    readRangesInfo();

    std::vector<float> ranges;
    float value;

    int num_emisison_readings;
    ifs >> num_emisison_readings;

    for (int i = 0; i < num_emisison_readings; i++) {
        ifs >> value;
        ranges.push_back(value);
    }

    laser_msg.intensities = ranges;

    double curTime;
    ifs >> curTime;
    laser_msg.header.stamp = ros::Time(curTime);
}


void Converter::handleRobotLaserMsg() {
    robot_link = links["ROBOT"];

    laser_msg.header.frame_id = robot_link;
    readRangesInfo();

    float info;
    for (int i = 0; i < 12; i++) // Don`t need next 12 fields.
        ifs >> info;

    double curTime;
    ifs >> curTime;
    laser_msg.header.stamp = ros::Time(curTime);
}


void Converter::incrementStamp() {
    stamp = stamp + ros::Duration(1 / rate);
}


void Converter::readRangesInfo() {
    float data1, angle_min, angle, angle_increment, range_max, data6, data7;
    ifs >> data1 >> angle_min >> angle >> angle_increment >> range_max >> data6 >> data7;

    laser_msg.angle_increment = angle_increment;
    laser_msg.angle_min = angle_min + laser_msg.angle_increment / 2;
    laser_msg.angle_max = angle_min + angle - laser_msg.angle_increment / 2;
    laser_msg.range_min = 0;
    laser_msg.range_max = range_max;

    std::vector<float> ranges;

    int num_range_readings;
    ifs >> num_range_readings;

    float value;
    for (int i = 0; i < num_range_readings; i++) {
        ifs >> value;
        ranges.push_back(value);
    }

    laser_msg.ranges = ranges;

    float factor_angle_fitting = laser_msg.angle_increment / 2;
    while ((round((laser_msg.angle_max - laser_msg.angle_min) / laser_msg.angle_increment) + 1) != num_range_readings) {
        if ((round((laser_msg.angle_max - laser_msg.angle_min) / laser_msg.angle_increment) + 1) < num_range_readings)
            laser_msg.angle_min = laser_msg.angle_min - factor_angle_fitting;

        else
            laser_msg.angle_max = laser_msg.angle_max - factor_angle_fitting;

        factor_angle_fitting = factor_angle_fitting / 2;
    }
}
