#include <cmath>
#include <stdlib.h>
#include <time.h>

#define RAND_MAX 100

//Function to cluster incoming GPS coordinates
void cluster_gps(std::vector<float64> lat_data, std::vector<float64> lon_data)  {
  //Initialize radius of windows
  int rad = 0.5;
  ////////////////
  //Normalize data
  ////////////////
  //Use initial data points as centroid
  std::vector<float64> lat_cent;
  std::vector<float64> lon_cent;
  while(1)  {
    //New centroids
    std::vector<float64> lat_new_cent;
    std::vector<float64> lon_new_cent;
    //For each centroid
    for(int cent_i=0; cent_i != lat_data.size(); cent_i++) {
      //Data points within range
      std::vector<float64> lat_within_range;
      std::vector<float64> lon_within_range;
      //Check distance of each data point
      for(int data_i=0; data_i != lat_data.size(); data_i++)  {
        //If within distance add it to "within range" list
        if(get_distance(lat_cent[cent_i]-lat_data[data_i],lon_cent[cent_i]-lon_data[data_i])<rad) {
          lat_within_range.push_back(lat_data[data_i]);
          lon_within_range.push_back(lon_data[data_i]);
        }
      }
      //New centroid is the mean of data within range
      lat_new_cent.push_back(get_average(lat_within_range));
      lon_new_cent.push_back(get_average(lon_within_range));
    }
    //Run until convergence
    //Check overlaps and select required windows
  }
}

//Function to get average of a vector
float64 get_average(std::vector<float64> inp) {
  float64 avg = 0f;
  for (int i=0; i<inp.size(); i++)  {
    avg += inp[i];
  }
  return avg/inp.size()
}

//Function to get distance between points
float64 get_distance(float64 lat_dis, float64 lon_dist)  {
  return sqrt(lat_dist*lat_dist + lon_dist*lon_dist);
}

//Function to check convergence
bool check_convergence()  {
  //Check number of data points within
}

//Main function at least for now
int main()  {
  //Initialize seed
  srand(time(0));

}
