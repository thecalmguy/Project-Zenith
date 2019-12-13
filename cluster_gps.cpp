#include <cmath>
#include <stdlib.h>
#include <time.h>

#define RAND_MAX 100

//Global variables
float64 avg_val[2];

//Function to cluster incoming GPS coordinates
void cluster_gps(std::vector<std::pair <float64, float64>> gps_data)  {
  //Initialize radius of windows
  int rad = 0.5;
  ////////////////
  //Normalize data
  ////////////////
  //Use initial data points as centroid
  std::vector<std::pair <float64, float64>> gps_cent;
  while(1)  {
    //New centroids
    std::vector<std::pair <float64, float64>> gps_new_cent;
    //For each centroid
    for(int cent_i=0; cent_i != gps_data.size(); cent_i++) {
      //Data points within range
      std::vector<std::pair <float64, float64>> gps_within_range;
      //Check distance of each data point
      for(int data_i=0; data_i != gps_data.size(); data_i++)  {
        //If within distance add it to "within range" list
        if(get_distance(gps_cent[cent_i].first-gps_data[data_i].first,gps_cent[cent_i].second-gps_data[data_i].second)<rad) {
          gps_within_range.push_back(lon_data[data_i]);
        }
      }
      //New centroid is the mean of data within range
      get_average(gps_within_range);
      gps_new_cent.push_back(avg_val);
      //To implement
      //Get the unique centroids of gps_new_cent
      //Check convergence
      //If yes, set gps_cent=gps_new_cent and break
      //If no, set gps_cent=gps_new_cent and run the loop

      // std::vector<int>::iterator ip;
      // ip = std::unique(gps_new_cent.begin(), gps_new_cent.end());
      // // Resizing the vector so as to remove the undefined terms
      // gps_new_cent.resize(std::distance(gps_new_cent.begin(), ip));

    }
    //Run until convergence
  }
}

//Function to get average of a vector
void get_average(std::vector<std::pair <float64, float64>> inp) {
  for (int i=0; i<inp.size(); i++)  {
    avg_val[0] += inp[i].first;
    avg_val[1] += inp[i].second;
  }
  avg_val[0] = avg_val[0]/inp.size();
  avg_val[1] = avg_val[1]/inp.size();
}

//Function to get distance between points
float64 get_distance(float64 lat_dis, float64 lon_dist)  {
  return sqrt(lat_dist*lat_dist + lon_dist*lon_dist);
}

//Function to check convergence
bool check_convergence()  {
  //Check if gps_new_cent and gps_cent have the same elements
}

//Main function at least for now
int main()  {
  //Initialize seed
  srand(time(0));

}
