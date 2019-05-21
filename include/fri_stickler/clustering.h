#ifndef CLUSTERING_H
#define CLUSTERING_H

#include <vector>

class PointCluster {
protected:
  std::vector<float> points;
  float cluster_mean, cluster_stdev;
  bool computed;

  void compute();

public:
  PointCluster();

  void add_point(float point);

  float mean();

  float stdev();

  int n();

  std::vector<float> get_points();
};

class PointClusters {
protected:
  std::vector<PointCluster> clusters;
  float z_max, p_err_max;

public:
  PointClusters(float z_max, float p_err_max);

  bool add_point(float point, bool virt);

  void summarize(bool verbose);

  PointCluster* get_largest_cluster();

  float rate_fit(PointCluster &cluster, float point);

  int num_clusters();
};

#endif
