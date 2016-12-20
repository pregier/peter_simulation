#include <iostream>
#include <fstream>
#include <random>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>

#include <cstdio>
#include <cerrno>
#include <sys/types.h>
#include <sys/stat.h>

using namespace std;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "object_file_creation_node");
  ros::NodeHandle node;

  //parameters for files
  string rootPath, folderName, fileName;
  vector<string> modelNames;
  int mapCount;
  //parameters for cluster
  int clusterCountMin, clusterCountMax;
  double clusterRadiusMin, clusterRadiusMax, clusterObjectDensityMin, clusterObjectDensityMax;
  double clusterXMin, clusterXMax, clusterYMin, clusterYMax;
  double clusterMinDistance;
  //parameters for spawning
  double spawnXMin, spawnXMax, spawnYMin, spawnYMax, spawnZMin, spawnZMax, spawnYawMin, spawnYawMax;
  //parameters for corner positions
  vector<vector<pair<double, double>>> objectCorners;
  //get params
  try
  {
    node.param<string>("root_path", rootPath, "/home/phil13131/workspaces/peter/clutter_ws/src/simulation_dev/simulated_navigation_launch/scripts/object_files_box_long");
    node.param<string>("folder_name", folderName, "cluster_count");
    node.param<string>("file_name", fileName, "object_location");
    node.param<int>("map_count", mapCount, 8);

    node.param<int>("cluster_count_min", clusterCountMin, 3);
    node.param<int>("cluster_count_max", clusterCountMax, 12);
    node.param<double>("cluster_radius_min", clusterRadiusMin, 1.5);
    node.param<double>("cluster_radius_max", clusterRadiusMax, 1.5);
    node.param<double>("cluster_object_density_min", clusterObjectDensityMin, 1.0);
    node.param<double>("cluster_object_density_max", clusterObjectDensityMax, 1.0);
    node.param<double>("cluster_xmin", clusterXMin, 4.5);
    node.param<double>("cluster_xmax", clusterXMax, 17.0);
    node.param<double>("cluster_ymin", clusterYMin, 0.2);
    node.param<double>("cluster_ymax", clusterYMax, 7.8);
    node.param<double>("cluster_min_distance", clusterMinDistance, 2*clusterRadiusMax);

    node.param<double>("spawn_xmin", spawnXMin, 3.5);
    node.param<double>("spawn_xmax", spawnXMax, 18.5);
    node.param<double>("spawn_ymin", spawnYMin, 0.2);
    node.param<double>("spawn_ymax", spawnYMax, 7.8);
    node.param<double>("spawn_zmin", spawnZMin, 0.0);
    node.param<double>("spawn_zmax", spawnZMax, 0.0);
    node.param<double>("spawn_yawmin", spawnYawMin, 0.0);
    node.param<double>("spawn_yawmax", spawnYawMax, 180.0);
    spawnYawMin *= 3.1415927 / 180.0;
    spawnYawMax *= 3.1415927 / 180.0;

    for(int i = 0; i < 5; ++i)
    {
      string model;
      node.param<string>("model_" + to_string(i) + "_name", model, "");
      if(model != "")
      {
    	  modelNames.push_back(model);
    	  double cornerX, cornerY;
    	  node.param<double>("object_" + to_string(i) + "_corner_x", cornerX, 0.0);
    	  node.param<double>("object_" + to_string(i) + "_corner_y", cornerY, 0.0);
    	  objectCorners.push_back(vector<pair<double, double>>(5, pair<double, double>(cornerX, cornerY)));
      }
      else
    	  continue;

      objectCorners.back()[1].first = objectCorners.back()[0].first;
      objectCorners.back()[1].second = -objectCorners.back()[0].second;
      objectCorners.back()[2].first = -objectCorners.back()[1].first;
      objectCorners.back()[2].second = objectCorners.back()[1].second;
      objectCorners.back()[3].first = objectCorners.back()[2].first;
      objectCorners.back()[3].second = -objectCorners.back()[2].second;
      objectCorners.back()[4].first = objectCorners.back()[0].first;
      objectCorners.back()[4].second = objectCorners.back()[0].second;
	}
  }
  catch (const ros::InvalidNameException &ex)
  {
    cerr << ex.what() << endl;
    return -1;
  }

  random_device rd;
  mt19937 rng(rd());

  uniform_real_distribution<double> distributionZ(spawnZMin, spawnZMax);
  uniform_real_distribution<double> distributionYaw(spawnYawMin, spawnYawMax);
  uniform_real_distribution<double> distributionXCluster(clusterXMin, clusterXMax);
  uniform_real_distribution<double> distributionYCluster(clusterYMin, clusterYMax);
  uniform_real_distribution<double> distributionDensity(clusterObjectDensityMin, clusterObjectDensityMax);
  uniform_real_distribution<double> distributionRadius(clusterRadiusMin, clusterRadiusMax);
  uniform_int_distribution<int> distributionModel(0, modelNames.size() -1);

  if (mkdir(rootPath.c_str(), S_IRWXU) == -1) {
      if (errno != EEXIST) {
          perror((string("Could not create directory ") + rootPath).c_str());
      }
  }


  //loop over current cluster count
  for (int clusterCount = clusterCountMin; clusterCount <= clusterCountMax; ++clusterCount)
  {
    clog << clusterCount << " clusters: " << endl;
    string dirPath = rootPath + "/" + folderName + "_" + to_string(clusterCount);
    if (mkdir(dirPath.c_str(), S_IRWXU) == -1) {
        if (errno != EEXIST) {
            perror((string("Could not create directory ") + dirPath).c_str());
            continue;
        }
    }
    string filePath = dirPath + "/" + fileName;
    //loop over files for a given cluster count
    for (int mapNumber = 1; mapNumber <= mapCount; ++mapNumber)
    {
      string filePathFullYaml = filePath + to_string(mapNumber) + ".yaml";
      string filePathFullTxt = filePath + to_string(mapNumber) + ".txt";


      int counterTrySpawn = 0, counterTryMap = 0;
      vector<pair<double, double>> clusterCenters;
      bool foundMap = true;
      while (clusterCenters.size() < clusterCount)
      {
    	  if(counterTrySpawn > 1000000)
    	  {
    		  ++counterTryMap;
    		  clusterCenters.clear();
    	  }
    	  if(counterTryMap > 1000)
    	  {
    		  foundMap = false;
    		  clog << "Could not find a valid cluster distribution for '" << filePath + to_string(mapNumber) + ".*" << "'." << endl;
    		  break;
    	  }

    	  double xCluster = distributionXCluster(rng);
    	  double yCluster = distributionYCluster(rng);
    	  bool isBad = false;
    	  for(int n = 0; n < clusterCenters.size(); ++n)
    		  if(pow(clusterCenters[n].first - xCluster, 2) + pow(clusterCenters[n].second - yCluster, 2) < clusterMinDistance*clusterMinDistance)
    		  {
    			  isBad = true;
    			  break;
    		  }
    	  if(isBad)
    	  {
    		  ++counterTrySpawn;
    		  continue;
    	  }

    	  counterTrySpawn = 0;
    	  clusterCenters.push_back(make_pair(xCluster, yCluster));
      }
      if(!foundMap)
    	  continue;

      ofstream fsYaml, fsTxt;
      fsYaml.open(filePathFullYaml, ios::out);
      fsTxt.open(filePathFullTxt, ios::out);
      if (!fsYaml.is_open() || !fsTxt.is_open())
      {
        clog << "Could not write to file '" << filePath + to_string(mapNumber) + ".*" << "'." << endl;
        fsYaml.close();
        fsTxt.close();
        continue;
      }

      clog << "Writing to map '" << filePath + to_string(mapNumber) + ".*" << "'" << endl;
      int totalObjectCounter = 0;

      //loop over all clusters for current file
      for (int cluster = 0; cluster < clusterCenters.size(); ++cluster)
      {
        double density = distributionDensity(rng);
        double radius = distributionRadius(rng);
        int objCountPerCluster = (int)(radius * radius * 3.1416 * density);
        double xCluster = clusterCenters[cluster].first;
        double yCluster = clusterCenters[cluster].second;
        uniform_real_distribution<double> distributionX(xCluster - radius, xCluster + radius);
        uniform_real_distribution<double> distributionY(yCluster - radius, yCluster + radius);

        int objectCounter = 0;

        //loop over all objects in current cluster
        while (objectCounter <= objCountPerCluster)
        {
          double x = distributionX(rng);
          double y = distributionY(rng);
          double z = distributionZ(rng);
          double yaw = distributionYaw(rng);
          int model = distributionModel(rng);

          if (x < spawnXMin || x > spawnXMax || y < spawnYMin || y > spawnYMax || (pow(xCluster - x, 2) + pow(yCluster - y, 2) > radius * radius))
            continue;

          fsYaml << "box_" << totalObjectCounter << ":" << endl;
          fsYaml << "  model: " << modelNames[model] << endl;
          fsYaml << "  model_type: urdf" << endl;
          fsYaml << "  position: [" << x << ", " << y << ", " << z << "]" << endl;
          fsYaml << "  orientation: [0.0, 0.0, " << yaw << "]" << endl;

          for(int corner = 0; corner <= 4; ++corner)
          {
             double xCorner = cos(yaw)*objectCorners[model][corner].first - sin(yaw)*objectCorners[model][corner].second + x;
             double yCorner = sin(yaw)*objectCorners[model][corner].first + cos(yaw)*objectCorners[model][corner].second + y;
             fsTxt << xCorner << " " << yCorner << " " << (cluster + 1) << endl;
          }
          fsTxt << endl;

          ++objectCounter;
          ++totalObjectCounter;
        }
      }

      fsYaml.close();
      fsTxt.close();
    }
  }

  clog << "done!" << endl;

  return 0;
}
