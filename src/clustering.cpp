#include "fri_stickler/clustering.h"

#include <cmath>
#include <iostream>
#include <math.h>
#include <ros/ros.h>

PointCluster::PointCluster() {
  computed = false;
}

void PointCluster::add_point(float point) {
  points.push_back(point);
  computed = false;
}

void PointCluster::compute() {
  float sigma_x = 0, sigma_x_squared = 0;
  int size = points.size();
  for (int i = 0; i < size; i++) {
    float p = points[i];
    sigma_x += p;
    sigma_x_squared += p * p;
  }

  cluster_mean = sigma_x / size;
  cluster_stdev = sqrt((sigma_x_squared - sigma_x * sigma_x / size) / size);
  computed = true;
}

float PointCluster::mean() {
  if (!computed)
    compute();

  return cluster_mean;
}

float PointCluster::stdev() {
  if (!computed)
    compute();

  return cluster_stdev;
}

int PointCluster::n() {
  return points.size();
}

std::vector<float> PointCluster::get_points() {
  return points;
}

PointClusters::PointClusters(float z_max, float p_err_max) {
  this->z_max = z_max;
  this->p_err_max = p_err_max;
}

bool PointClusters::add_point(float point, bool virt) {
  if (clusters.size() == 0) {
    clusters.push_back(PointCluster());
    clusters[0].add_point(point);
    return false;
  }

  float best_cluster_fit = -1;
  int best_cluster_ind = -1;

  for (int i = 0; i < clusters.size(); i++) {
    float fit = rate_fit(clusters[i], point);
    if (!std::isnan(fit) && (best_cluster_fit == -1 ||
        fit < best_cluster_fit)) {
      best_cluster_fit = fit;
      best_cluster_ind = i;
    }
  }

  if (best_cluster_ind != -1) {
    if (!virt)
      clusters[best_cluster_ind].add_point(point);
    return true;
  } else {
    if (!virt) {
      PointCluster new_cluster;
      new_cluster.add_point(point);
      clusters.push_back(new_cluster);
    }
    return false;
  }
}

float PointClusters::rate_fit(PointCluster &cluster, float point) {
  float z_score = (cluster.mean() - point) / cluster.stdev();
  float p_err = (point - cluster.mean()) / cluster.mean();
  if (fabs(z_score) < z_max || fabs(p_err) < p_err_max)
    return z_score + p_err;
  return NAN;
}

PointCluster* PointClusters::get_largest_cluster() {
  int largest_ind = -1;
  for (int i = 0; i < clusters.size(); i++)
    if (largest_ind == -1 || clusters[i].n() > clusters[largest_ind].n())
      largest_ind = i;
  return largest_ind == -1 ? nullptr : &clusters[largest_ind];
}

void PointClusters::summarize(bool verbose) {
  for (int i = 0; i < clusters.size(); i++) {
    PointCluster c = clusters[i];
    ROS_INFO("Cluster %d; u=%f, s=%f, n=%d", i, c.mean(), c.stdev(), c.n());
    if (verbose) {
      std::vector<float> points = c.get_points();
      for (int j = 0; j < points.size(); j++)
        std::cout << points[j] << " ";
      std::cout << std::endl;
    }
  }
}

int PointClusters::num_clusters() {
  return clusters.size();
}
