#ifndef CLUSTERING_H
#define CLUSTERING_H

#include <vector>

/**
  A cluster of 1D points.
*/
class PointCluster {
protected:
  std::vector<float> points;
  float cluster_mean, cluster_stdev;
  bool computed;

  /**
    @brief Computes mean and stdev of contained points
  */
  void compute();

public:
  /**
    @brief Creates an empty point cluster
  */
  PointCluster();

  /**
    @brief Adds a point to this cluster
  */
  void add_point(float point);

  /**
    @brief Gets the mean of this cluster's points
  */
  float mean();

  /**
    @brief Gets the standard deviation of this cluster's points
  */
  float stdev();

  /**
    @brief Gets the number of points in this cluster
  */
  int n();

  /**
    @brief Gets a copy of the vector of points in this cluster
  */
  std::vector<float> get_points();
};

/**
  A set of clusters with rules defining how points are split between them
*/
class PointClusters {
protected:
  std::vector<PointCluster> clusters;
  float z_max, p_err_max;

public:
  /**
    Creates an empty cluster set.

    @param z_max maximum z score a point can have to join a cluster
    @param p_err_max maximum percent error a point can have to join a cluster
  */
  PointClusters(float z_max, float p_err_max);

  /**
    Adds a point to this set of clusters. The point may join a pre-existing
    cluster or become the start of a new cluster.

    @param point value
    @param virt virtual add
    @return if the point joined a pre-existing cluster
  */
  bool add_point(float point, bool virt);

  /**
    @brief Prints a statistical summary of the cluster for debugging purposes
  */
  void summarize(bool verbose);

  /**
    @brief Gets a pointer to the largest cluster in the set, or nullptr if
      the set is empty
  */
  PointCluster* get_largest_cluster();

  /**
    @brief Computes a fit score for a point and a cluster
  */
  float rate_fit(PointCluster &cluster, float point);

  /**
    @brief Gets the number of clusters in the set
  */
  int num_clusters();
};

#endif
