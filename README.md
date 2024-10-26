

when `pcl::visualization::PCLVisualizer::Ptr viewer` is dependent, logo and color list can be loaded normally. However if embed it onto `QVTKWidget *qvtkWidget`, they will disapear. It's not solved by now. I think the key is "interactor()". 