#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <cv_bridge/cv_bridge.h>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/stlemitter.h>
#include <yaml-cpp/emitter.h>
#include <boost/filesystem.hpp>

#include <random>
#include <cstdlib>

void object_names(std::vector<std::string>& object_name)
{
  object_name.resize(19);

  object_name[0] = "salt";
  object_name[1] = "tomato_soup";
  object_name[2] = "tomato_sauce";
  object_name[3] = "zwieback";
  object_name[4] = "basmati_rice";
  object_name[5] = "spaghetti";
  object_name[6] = "orange_juice_hohesc";
  object_name[7] = "green_tea";
  object_name[8] = "milk";
  object_name[9] = "candle_thin";
  object_name[10] = "candle_thick_holder";
  object_name[11] = "book_romantiker";
  object_name[12] = "book_gardening";
  object_name[13] = "plastic_wrap";
  object_name[14] = "kitekat";
  object_name[15] = "picture_family";
  object_name[16] = "book_pferdewitze";
  object_name[17] = "book_heilkrauter";
  object_name[18] = "box_wood";
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "set_clutter");
  ros::NodeHandle nh("~");

  std::vector<std::vector<float>> position_vector;
  std::vector<std::vector<float>> orientation_vector;
  double minx, miny, maxx, maxy;
  int numOfObjects;
  double min_distance;
  std::string file_location;

  if (!nh.getParam("minx", minx))
  {
    minx = 40.0;
    ROS_INFO("The Parameter minx is not set. It set to default value %f", minx);
  }
  else
  {
    ROS_INFO("minx is set to %f", minx);
  }

  if (!nh.getParam("miny", miny))
  {
    miny = 32.0;
    ROS_INFO("The Parameter miny is not set. It set to default value %f", miny);
  }
  else
  {
    ROS_INFO("miny is set to %f", miny);
  }

  if (!nh.getParam("maxx", maxx))
  {
    maxx = 48.0;
    ROS_INFO("The Parameter maxx is not set. It set to default value %f", maxx);
  }
  else
  {
    ROS_INFO("maxx is set to %f", maxx);
  }

  if (!nh.getParam("maxy", maxy))
  {
    maxy = 36.0;
    ROS_INFO("The Parameter maxy is not set. It set to default value %f", maxy);
  }
  else
  {
    ROS_INFO("maxy is set to %f", maxy);
  }

  if (!nh.getParam("numOfObjects", numOfObjects))
  {
    numOfObjects = 30;
    ROS_INFO("The Parameter numOfObjects is not set. It set to default value %d", numOfObjects);
  }
  else
  {
    ROS_INFO("numOfObject is set to %d", numOfObjects);
  }

  if (!nh.getParam("min_distance", min_distance))
  {
    min_distance = 0.35;
    ROS_INFO("The Parameter min_distance is not set. It set to default value %f", min_distance);
  }
  else
  {
    ROS_INFO("min_distance is set to %f", min_distance);
  }

  if (!nh.getParam("file_location", file_location))
  {
    file_location =
        "/home/pregier/Workspaces/clutter_navigation_ws/src/simulation_dev/simulated_objects/object_config/object_location.yaml";
    ROS_INFO("The Parameter file_location is not set. It set to default value %s", file_location.c_str());
  }
  else
  {
    ROS_INFO("file_location is set to %s", file_location.c_str());
  }

  srand(time(NULL));
  const char char_set[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
      "abcdefghijklmnopqrstuvwxyz";
  std::vector<std::string> object_name;
  object_names(object_name);

  while (numOfObjects > position_vector.size())
  {
    float x = rand() % (int)(maxx - minx) + minx;
    x += (rand() % 100) * 0.01;

    float y = rand() % (int)(maxy - miny) + miny;
    y += (rand() % 100) * 0.01;

    bool point_to_close = false;
    for (int i = 0; i < position_vector.size(); ++i)
    {
      if (sqrt(pow(x - position_vector[i][0], 2) + pow(y - position_vector[i][1], 2)) < min_distance)
      {
        point_to_close = true;
        break;
      }
    }
    if (!point_to_close)
    {
      std::vector<float> temp_pos {x, y, 0.3};
      position_vector.push_back(temp_pos);
    }
  }

  while (numOfObjects > orientation_vector.size())
  {
    float xr = rand() % 181;
    if (xr > 90)
      xr = xr * 3.14592 / 180;
    else
      xr = -xr * 3.14592 / 180;

    float yr = rand() % 181;
    if (yr > 90)
      yr = yr * 3.14592 / 180;
    else
      yr = -yr * 3.14592 / 180;

    float zr = rand() % 181;
    if (zr > 90)
      zr = zr * 3.14592 / 180;
    else
      zr = -zr * 3.14592 / 180;
    std::vector<float> temp_or {xr, yr, zr};
    orientation_vector.push_back(temp_or);
  }

  std::vector<std::pair<std::string, int>> name_buffer;

  YAML::Node node;
  YAML::Emitter out;
  out << YAML::BeginMap;
  for (int i = 0; i < numOfObjects; ++i)
  {
    int objectNumber = rand() % 19;

    std::string name = object_name[objectNumber];
    bool equalString = false;
    for (int i = 0; i < name_buffer.size(); ++i)
    {
      if (!std::strcmp(name_buffer[i].first.c_str(), name.c_str()))
      {
        name += "_";
        name += char_set[name_buffer[i].second];
        name_buffer[i].second++;
        equalString = true;
        break;
      }
    }
    if (!equalString)
    {
      name_buffer.push_back(std::make_pair(name, 0));
    }

    out << YAML::Key << name;
    out << YAML::BeginMap;
    out << YAML::Key << "model";
    out << YAML::Value << object_name[objectNumber];
    out << YAML::Key << "model_type";
    out << YAML::Value << "urdf";
    out << YAML::Key << "position";
    out << YAML::Flow << position_vector[i];
    out << YAML::Key << "orientation";
    out << YAML::Flow << orientation_vector[i];
    out << YAML::EndMap;

  }
  out << YAML::EndMap;
  std::ofstream fout(file_location);
  fout << "# Bounding box of the cluttered area with " << numOfObjects << " objects: \n" << "# X-Range " << minx << " - " << maxx << ";\n# Y-Range "
      << miny << " - " << maxy << "; \n\n";
  fout << out.c_str();
  fout.close();
  return 0;
}

