Help on package open3d:

NAME
    open3d

FILE
    /home/okano/.local/lib/python2.7/site-packages/open3d/__init__.py

DESCRIPTION
    # Open3D: www.open3d.org
    # The MIT License (MIT)
    # See license file or visit www.open3d.org for details

PACKAGE CONTENTS
    linux (package)
    macos (package)
    win32 (package)

SUBMODULES
    open3d

FUNCTIONS
    color_map_optimization(...)
        color_map_optimization(mesh: open3d.linux.open3d.TriangleMesh, imgs_rgbd: List[open3d.linux.open3d.RGBDImage], camera: open3d.linux.open3d.PinholeCameraTrajectory, option: open3d.linux.open3d.ColorMapOptmizationOption=ColorMapOptmizationOption with
        - non_rigid_camera_coordinate : 0
        - number_of_vertical_anchors : 16
        - non_rigid_anchor_point_weight : 0.316000
        - maximum_iteration : 300.000000
        - maximum_allowable_depth : 2.500000
        - depth_threshold_for_visiblity_check : 0.030000) -> None
        
        Function for color mapping of reconstructed scenes via optimization
    
    compute_fpfh_feature(...)
        compute_fpfh_feature(input: open3d.linux.open3d.PointCloud, search_param: open3d.linux.open3d.KDTreeSearchParam) -> open3d.linux.open3d.Feature
        
        Function to compute FPFH feature for a point cloud
    
    compute_point_cloud_mahalanobis_distance(...)
        compute_point_cloud_mahalanobis_distance(input: open3d.linux.open3d.PointCloud) -> open3d.linux.open3d.DoubleVector
        
        Function to compute the Mahalanobis distance for points in a point cloud
    
    compute_point_cloud_mean_and_covariance(...)
        compute_point_cloud_mean_and_covariance(input: open3d.linux.open3d.PointCloud) -> Tuple[numpy.ndarray[float64[3, 1]], numpy.ndarray[float64[3, 3]]]
        
        Function to compute the mean and covariance matrix of a point cloud
    
    compute_point_cloud_nearest_neighbor_distance(...)
        compute_point_cloud_nearest_neighbor_distance(input: open3d.linux.open3d.PointCloud) -> open3d.linux.open3d.DoubleVector
        
        Function to compute the distance from a point to its nearest neighbor in the point cloud
    
    compute_point_cloud_to_point_cloud_distance(...)
        compute_point_cloud_to_point_cloud_distance(source: open3d.linux.open3d.PointCloud, target: open3d.linux.open3d.PointCloud) -> open3d.linux.open3d.DoubleVector
        
        Function to compute the ponit to point distances between point clouds
    
    compute_rgbd_odometry(...)
        compute_rgbd_odometry(rgbd_source: open3d.linux.open3d.RGBDImage, rgbd_target: open3d.linux.open3d.RGBDImage, pinhole_camera_intrinsic: open3d.linux.open3d.PinholeCameraIntrinsic=PinholeCameraIntrinsic with width = -1 and height = -1.
        Access intrinsics with intrinsic_matrix., odo_init: numpy.ndarray[float64[4, 4]]=array([[ 1.,  0.,  0.,  0.],
               [ 0.,  1.,  0.,  0.],
               [ 0.,  0.,  1.,  0.],
               [ 0.,  0.,  0.,  1.]]), jacobian: open3d.linux.open3d.RGBDOdometryJacobian=RGBDOdometryJacobianFromHybridTerm, option: open3d.linux.open3d.OdometryOption=OdometryOption class.
        iteration_number_per_pyramid_level = [ 20, 10, 5, ] 
        max_depth_diff = 0.030000
        min_depth = 0.000000
        max_depth = 4.000000) -> Tuple[bool, numpy.ndarray[float64[4, 4]], numpy.ndarray[float64[6, 6]]]
        
        Function to estimate 6D rigid motion from two RGBD image pairs
    
    create_image_pyramid(...)
        create_image_pyramid(image: open3d.linux.open3d.Image, num_of_levels: int, with_gaussian_filter: bool) -> List[open3d.linux.open3d.Image]
        
        Function to create ImagePyramid
    
    create_mesh_arrow(...)
        create_mesh_arrow(cylinder_radius: float=1.0, cone_radius: float=1.5, cylinder_height: float=5.0, cone_height: float=4.0, resolution: int=20L, cylinder_split: int=4L, cone_split: int=1L) -> open3d.linux.open3d.TriangleMesh
        
        Factory function to create an arrow mesh
    
    create_mesh_cone(...)
        create_mesh_cone(radius: float=1.0, height: float=2.0, resolution: int=20L, split: int=1L) -> open3d.linux.open3d.TriangleMesh
        
        Factory function to create a cone mesh
    
    create_mesh_coordinate_frame(...)
        create_mesh_coordinate_frame(size: float=1.0, origin: numpy.ndarray[float64[3, 1]]=array([ 0.,  0.,  0.])) -> open3d.linux.open3d.TriangleMesh
        
        Factory function to create a coordinate frame mesh
    
    create_mesh_cylinder(...)
        create_mesh_cylinder(radius: float=1.0, height: float=2.0, resolution: int=20L, split: int=4L) -> open3d.linux.open3d.TriangleMesh
        
        Factory function to create a cylinder mesh
    
    create_mesh_sphere(...)
        create_mesh_sphere(radius: float=1.0, resolution: int=20L) -> open3d.linux.open3d.TriangleMesh
        
        Factory function to create a sphere mesh
    
    create_point_cloud_from_depth_image(...)
        create_point_cloud_from_depth_image(depth: open3d.linux.open3d.Image, intrinsic: open3d.linux.open3d.PinholeCameraIntrinsic, extrinsic: numpy.ndarray[float64[4, 4]]=array([[ 1.,  0.,  0.,  0.],
               [ 0.,  1.,  0.,  0.],
               [ 0.,  0.,  1.,  0.],
               [ 0.,  0.,  0.,  1.]]), depth_scale: float=1000.0, depth_trunc: float=1000.0, stride: int=1L) -> open3d.linux.open3d.PointCloud
        
        Factory function to create a pointcloud from a depth image and a camera.
        Given depth value d at (u, v) image coordinate, the corresponding 3d point is:
            z = d / depth_scale
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
    
    create_point_cloud_from_rgbd_image(...)
        create_point_cloud_from_rgbd_image(image: open3d.linux.open3d.RGBDImage, intrinsic: open3d.linux.open3d.PinholeCameraIntrinsic, extrinsic: numpy.ndarray[float64[4, 4]]=array([[ 1.,  0.,  0.,  0.],
               [ 0.,  1.,  0.,  0.],
               [ 0.,  0.,  1.,  0.],
               [ 0.,  0.,  0.,  1.]])) -> open3d.linux.open3d.PointCloud
        
        Factory function to create a pointcloud from an RGB-D image and a camera.
        Given depth value d at (u, v) image coordinate, the corresponding 3d point is:
            z = d / depth_scale
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
    
    create_rgbd_image_from_color_and_depth(...)
        create_rgbd_image_from_color_and_depth(color: open3d.linux.open3d.Image, depth: open3d.linux.open3d.Image, depth_scale: float=1000.0, depth_trunc: float=3.0, convert_rgb_to_intensity: bool=True) -> open3d.linux.open3d.RGBDImage
        
        Function to make RGBDImage
    
    create_rgbd_image_from_nyu_format(...)
        create_rgbd_image_from_nyu_format(color: open3d.linux.open3d.Image, depth: open3d.linux.open3d.Image, convert_rgb_to_intensity: bool=True) -> open3d.linux.open3d.RGBDImage
        
        Function to make RGBDImage (for NYU format)
    
    create_rgbd_image_from_sun_format(...)
        create_rgbd_image_from_sun_format(color: open3d.linux.open3d.Image, depth: open3d.linux.open3d.Image, convert_rgb_to_intensity: bool=True) -> open3d.linux.open3d.RGBDImage
        
        Function to make RGBDImage (for SUN format)
    
    create_rgbd_image_from_tum_format(...)
        create_rgbd_image_from_tum_format(color: open3d.linux.open3d.Image, depth: open3d.linux.open3d.Image, convert_rgb_to_intensity: bool=True) -> open3d.linux.open3d.RGBDImage
        
        Function to make RGBDImage (for TUM format)
    
    crop_point_cloud(...)
        crop_point_cloud(input: open3d.linux.open3d.PointCloud, min_bound: numpy.ndarray[float64[3, 1]], max_bound: numpy.ndarray[float64[3, 1]]) -> open3d.linux.open3d.PointCloud
        
        Function to crop input pointcloud into output pointcloud
    
    crop_triangle_mesh(...)
        crop_triangle_mesh(input: open3d.linux.open3d.TriangleMesh, min_bound: numpy.ndarray[float64[3, 1]], max_bound: numpy.ndarray[float64[3, 1]]) -> open3d.linux.open3d.TriangleMesh
        
        Function to crop input triangle mesh into output triangle mesh
    
    draw_geometries(...)
        draw_geometries(geometry_list: List[open3d.linux.open3d.Geometry], window_name: unicode=u'Open3D', width: int=1920L, height: int=1080L, left: int=50L, top: int=50L) -> None
        
        Function to draw a list of Geometry objects
    
    draw_geometries_with_animation_callback(...)
        draw_geometries_with_animation_callback(geometry_list: List[open3d.linux.open3d.Geometry], callback_function: Callable[[open3d.linux.open3d.Visualizer], bool], window_name: unicode=u'Open3D', width: int=1920L, height: int=1080L, left: int=50L, top: int=50L) -> None
        
        Function to draw a list of Geometry objects with a customized animation callback function
    
    draw_geometries_with_custom_animation(...)
        draw_geometries_with_custom_animation(geometry_list: List[open3d.linux.open3d.Geometry], window_name: unicode=u'Open3D', width: int=1920L, height: int=1080L, left: int=50L, top: int=50L, optional_view_trajectory_json_file: unicode=u'') -> None
        
        Function to draw a list of Geometry objects with a GUI that supports animation
    
    draw_geometries_with_editing(...)
        draw_geometries_with_editing(geometry_list: List[open3d.linux.open3d.Geometry], window_name: unicode=u'Open3D', width: int=1920L, height: int=1080L, left: int=50L, top: int=50L) -> None
        
        Function to draw a list of Geometry providing user interaction
    
    draw_geometries_with_key_callbacks(...)
        draw_geometries_with_key_callbacks(geometry_list: List[open3d.linux.open3d.Geometry], key_to_callback: Dict[int, Callable[[open3d.linux.open3d.Visualizer], bool]], window_name: unicode=u'Open3D', width: int=1920L, height: int=1080L, left: int=50L, top: int=50L) -> None
        
        Function to draw a list of Geometry objects with a customized key-callback mapping
    
    estimate_normals(...)
        estimate_normals(cloud: open3d.linux.open3d.PointCloud, search_param: open3d.linux.open3d.KDTreeSearchParam=KDTreeSearchParamKNN with knn = 30) -> bool
        
        Function to compute the normals of a point cloud
    
    evaluate_registration(...)
        evaluate_registration(source: open3d.linux.open3d.PointCloud, target: open3d.linux.open3d.PointCloud, max_correspondence_distance: float, transformation: numpy.ndarray[float64[4, 4]]=array([[ 1.,  0.,  0.,  0.],
               [ 0.,  1.,  0.,  0.],
               [ 0.,  0.,  1.,  0.],
               [ 0.,  0.,  0.,  1.]])) -> open3d.linux.open3d.RegistrationResult
        
        Function for evaluating registration between point clouds
    
    filter_image(...)
        filter_image(image: open3d.linux.open3d.Image, filter_type: open3d.linux.open3d.ImageFilterType) -> open3d.linux.open3d.Image
        
        Function to filter Image
    
    filter_image_pyramid(...)
        filter_image_pyramid(image_pyramid: List[open3d.linux.open3d.Image], filter_type: open3d.linux.open3d.ImageFilterType) -> List[open3d.linux.open3d.Image]
        
        Function to filter ImagePyramid
    
    get_information_matrix_from_point_clouds(...)
        get_information_matrix_from_point_clouds(source: open3d.linux.open3d.PointCloud, target: open3d.linux.open3d.PointCloud, max_correspondence_distance: float, transformation_result: numpy.ndarray[float64[4, 4]]) -> numpy.ndarray[float64[6, 6]]
        
        Function for computing information matrix from RegistrationResult
    
    get_verbosity_level(...)
        get_verbosity_level() -> open3d.linux.open3d.VerbosityLevel
        
        Get global verbosity level of open3d
    
    global_optimization(...)
        global_optimization(pose_graph: open3d.linux.open3d.PoseGraph, method: open3d.linux.open3d.GlobalOptimizationMethod, criteria: open3d.linux.open3d.GlobalOptimizationConvergenceCriteria, option: open3d.linux.open3d.GlobalOptimizationOption) -> None
        
        Function to optimize PoseGraph
    
    orient_normals_to_align_with_direction(...)
        orient_normals_to_align_with_direction(cloud: open3d.linux.open3d.PointCloud, orientation_reference: numpy.ndarray[float64[3, 1]]=array([ 0.,  0.,  1.])) -> bool
        
        Function to orient the normals of a point cloud
    
    orient_normals_towards_camera_location(...)
        orient_normals_towards_camera_location(cloud: open3d.linux.open3d.PointCloud, camera_location: numpy.ndarray[float64[3, 1]]=array([ 0.,  0.,  0.])) -> bool
        
        Function to orient the normals of a point cloud
    
    read_feature(...)
        read_feature(filename: unicode) -> open3d.linux.open3d.Feature
        
        Function to read Feature from file
    
    read_image(...)
        read_image(filename: unicode) -> open3d.linux.open3d.Image
        
        Function to read Image from file
    
    read_pinhole_camera_intrinsic(...)
        read_pinhole_camera_intrinsic(filename: unicode) -> open3d.linux.open3d.PinholeCameraIntrinsic
        
        Function to read PinholeCameraIntrinsic from file
    
    read_pinhole_camera_trajectory(...)
        read_pinhole_camera_trajectory(filename: unicode) -> open3d.linux.open3d.PinholeCameraTrajectory
        
        Function to read PinholeCameraTrajectory from file
    
    read_point_cloud(...)
        read_point_cloud(filename: unicode) -> open3d.linux.open3d.PointCloud
        
        Function to read PointCloud from file
    
    read_pose_graph(...)
        read_pose_graph(filename: unicode) -> open3d.linux.open3d.PoseGraph
        
        Function to read PoseGraph from file
    
    read_selection_polygon_volume(...)
        read_selection_polygon_volume(filename: unicode) -> open3d.linux.open3d.SelectionPolygonVolume
        
        Function to read SelectionPolygonVolume from file
    
    read_triangle_mesh(...)
        read_triangle_mesh(filename: unicode) -> open3d.linux.open3d.TriangleMesh
        
        Function to read TriangleMesh from file
    
    registration_colored_icp(...)
        registration_colored_icp(source: open3d.linux.open3d.PointCloud, target: open3d.linux.open3d.PointCloud, max_correspondence_distance: float, init: numpy.ndarray[float64[4, 4]]=array([[ 1.,  0.,  0.,  0.],
               [ 0.,  1.,  0.,  0.],
               [ 0.,  0.,  1.,  0.],
               [ 0.,  0.,  0.,  1.]]), criteria: open3d.linux.open3d.ICPConvergenceCriteria=ICPConvergenceCriteria class with relative_fitness = 0.000001, relative_rmse = 0.000001, and max_iteration = 30, lambda_geometric: float=0.968) -> open3d.linux.open3d.RegistrationResult
        
        Function for Colored ICP registration
    
    registration_fast_based_on_feature_matching(...)
        registration_fast_based_on_feature_matching(source: open3d.linux.open3d.PointCloud, target: open3d.linux.open3d.PointCloud, source_feature: open3d.linux.open3d.Feature, target_feature: open3d.linux.open3d.Feature, option: open3d.linux.open3d.FastGlobalRegistrationOption=FastGlobalRegistrationOption class with 
        division_factor = 1.400000
        use_absolute_scale = 0
        decrease_mu = 150
        maximum_correspondence_distance = 0.000000
        iteration_number = 64
        tuple_scale = 0.950000
        maximum_tuple_count = 1000) -> open3d.linux.open3d.RegistrationResult
        
        Function for fast global registration based on feature matching
    
    registration_icp(...)
        registration_icp(source: open3d.linux.open3d.PointCloud, target: open3d.linux.open3d.PointCloud, max_correspondence_distance: float, init: numpy.ndarray[float64[4, 4]]=array([[ 1.,  0.,  0.,  0.],
               [ 0.,  1.,  0.,  0.],
               [ 0.,  0.,  1.,  0.],
               [ 0.,  0.,  0.,  1.]]), estimation_method: open3d.linux.open3d.TransformationEstimation=TransformationEstimationPointToPoint without scaling., criteria: open3d.linux.open3d.ICPConvergenceCriteria=ICPConvergenceCriteria class with relative_fitness = 0.000001, relative_rmse = 0.000001, and max_iteration = 30) -> open3d.linux.open3d.RegistrationResult
        
        Function for ICP registration
    
    registration_ransac_based_on_correspondence(...)
        registration_ransac_based_on_correspondence(source: open3d.linux.open3d.PointCloud, target: open3d.linux.open3d.PointCloud, corres: open3d.linux.open3d.Vector2iVector, max_correspondence_distance: float, estimation_method: open3d.linux.open3d.TransformationEstimation=TransformationEstimationPointToPoint without scaling., ransac_n: int=6L, criteria: open3d.linux.open3d.RANSACConvergenceCriteria=RANSACConvergenceCriteria class with max_iteration = 1000, and max_validation = 1000) -> open3d.linux.open3d.RegistrationResult
        
        Function for global RANSAC registration based on a set of correspondences
    
    registration_ransac_based_on_feature_matching(...)
        registration_ransac_based_on_feature_matching(source: open3d.linux.open3d.PointCloud, target: open3d.linux.open3d.PointCloud, source_feature: open3d.linux.open3d.Feature, target_feature: open3d.linux.open3d.Feature, max_correspondence_distance: float, estimation_method: open3d.linux.open3d.TransformationEstimation=TransformationEstimationPointToPoint without scaling., ransac_n: int=4L, checkers: List[open3d.linux.open3d.CorrespondenceChecker]=[], criteria: open3d.linux.open3d.RANSACConvergenceCriteria=RANSACConvergenceCriteria class with max_iteration = 100000, and max_validation = 100) -> open3d.linux.open3d.RegistrationResult
        
        Function for global RANSAC registration based on feature matching
    
    select_down_sample(...)
        select_down_sample(*args, **kwargs)
        Overloaded function.
        
        1. select_down_sample(input: open3d.linux.open3d.PointCloud, indices: List[int]) -> open3d.linux.open3d.PointCloud
        
        Function to select points from input pointcloud into output pointcloud
        
        2. select_down_sample(input: open3d.linux.open3d.TriangleMesh, indices: List[int]) -> open3d.linux.open3d.TriangleMesh
        
        Function to select mesh from input triangle mesh into output triangle mesh
    
    set_verbosity_level(...)
        set_verbosity_level(verbosity_level: open3d.linux.open3d.VerbosityLevel) -> None
        
        Set global verbosity level of open3d
    
    uniform_down_sample(...)
        uniform_down_sample(input: open3d.linux.open3d.PointCloud, every_k_points: int) -> open3d.linux.open3d.PointCloud
        
        Function to downsample input pointcloud into output pointcloud uniformly
    
    voxel_down_sample(...)
        voxel_down_sample(input: open3d.linux.open3d.PointCloud, voxel_size: float) -> open3d.linux.open3d.PointCloud
        
        Function to downsample input pointcloud into output pointcloud with a voxel
    
    write_feature(...)
        write_feature(filename: unicode, feature: open3d.linux.open3d.Feature) -> bool
        
        Function to write Feature to file
    
    write_image(...)
        write_image(filename: unicode, image: open3d.linux.open3d.Image, quality: int=90L) -> bool
        
        Function to write Image to file
    
    write_pinhole_camera_intrinsic(...)
        write_pinhole_camera_intrinsic(filename: unicode, intrinsic: open3d.linux.open3d.PinholeCameraIntrinsic) -> bool
        
        Function to write PinholeCameraIntrinsic to file
    
    write_pinhole_camera_trajectory(...)
        write_pinhole_camera_trajectory(filename: unicode, trajectory: open3d.linux.open3d.PinholeCameraTrajectory) -> bool
        
        Function to write PinholeCameraTrajectory to file
    
    write_point_cloud(...)
        write_point_cloud(filename: unicode, pointcloud: open3d.linux.open3d.PointCloud, write_ascii: bool=False, compressed: bool=False) -> bool
        
        Function to write PointCloud to file
    
    write_pose_graph(...)
        write_pose_graph(filename: unicode, pose_graph: open3d.linux.open3d.PoseGraph) -> None
        
        Function to write PoseGraph to file
    
    write_triangle_mesh(...)
        write_triangle_mesh(filename: unicode, mesh: open3d.linux.open3d.TriangleMesh, write_ascii: bool=False, compressed: bool=False) -> bool
        
        Function to write TriangleMesh to file

DATA
    Always = VerbosityLevel.Always
    Debug = VerbosityLevel.Debug
    Error = VerbosityLevel.Error
    Gaussian3 = ImageFilterType.Gaussian3
    Gaussian5 = ImageFilterType.Gaussian5
    Gaussian7 = ImageFilterType.Gaussian7
    Gray32 = TSDFVolumeColorType.Gray32
    Info = VerbosityLevel.Info
    Kinect2ColorCameraDefault = PinholeCameraIntrinsicParameters.Kinect2Co...
    Kinect2DepthCameraDefault = PinholeCameraIntrinsicParameters.Kinect2De...
    None = TSDFVolumeColorType.None
    PrimeSenseDefault = PinholeCameraIntrinsicParameters.PrimeSenseDefault
    RGB8 = TSDFVolumeColorType.RGB8
    Sobel3dx = ImageFilterType.Sobel3dx
    Sobel3dy = ImageFilterType.Sobel3dy
    Warning = VerbosityLevel.Warning


Help on class PointCloud in module open3d.linux.open3d:

class PointCloud(Geometry3D)
 |  Method resolution order:
 |      PointCloud
 |      Geometry3D
 |      Geometry
 |      pybind11_builtins.pybind11_object
 |      __builtin__.object
 |  
 |  Methods defined here:
 |  
 |  __add__(...)
 |      __add__(self: open3d.linux.open3d.PointCloud, arg0: open3d.linux.open3d.PointCloud) -> open3d.linux.open3d.PointCloud
 |  
 |  __copy__(...)
 |      __copy__(self: open3d.linux.open3d.PointCloud) -> open3d.linux.open3d.PointCloud
 |  
 |  __deepcopy__(...)
 |      __deepcopy__(self: open3d.linux.open3d.PointCloud, arg0: dict) -> open3d.linux.open3d.PointCloud
 |  
 |  __iadd__(...)
 |      __iadd__(self: open3d.linux.open3d.PointCloud, arg0: open3d.linux.open3d.PointCloud) -> open3d.linux.open3d.PointCloud
 |  
 |  __init__(...)
 |      __init__(*args, **kwargs)
 |      Overloaded function.
 |      
 |      1. __init__(self: open3d.linux.open3d.PointCloud) -> None
 |      
 |      Default constructor
 |      
 |      2. __init__(self: open3d.linux.open3d.PointCloud, arg0: open3d.linux.open3d.PointCloud) -> None
 |      
 |      Copy constructor
 |  
 |  __repr__(...)
 |      __repr__(self: open3d.linux.open3d.PointCloud) -> unicode
 |  
 |  has_colors(...)
 |      has_colors(self: open3d.linux.open3d.PointCloud) -> bool
 |  
 |  has_normals(...)
 |      has_normals(self: open3d.linux.open3d.PointCloud) -> bool
 |  
 |  has_points(...)
 |      has_points(self: open3d.linux.open3d.PointCloud) -> bool
 |  
 |  normalize_normals(...)
 |      normalize_normals(self: open3d.linux.open3d.PointCloud) -> None
 |  
 |  paint_uniform_color(...)
 |      paint_uniform_color(self: open3d.linux.open3d.PointCloud, arg0: numpy.ndarray[float64[3, 1]]) -> None
 |  
 |  ----------------------------------------------------------------------
 |  Data descriptors defined here:
 |  
 |  colors
 |  
 |  normals
 |  
 |  points
 |  
 |  ----------------------------------------------------------------------
 |  Methods inherited from Geometry3D:
 |  
 |  get_max_bound(...)
 |      get_max_bound(self: open3d.linux.open3d.Geometry3D) -> numpy.ndarray[float64[3, 1]]
 |  
 |  get_min_bound(...)
 |      get_min_bound(self: open3d.linux.open3d.Geometry3D) -> numpy.ndarray[float64[3, 1]]
 |  
 |  transform(...)
 |      transform(self: open3d.linux.open3d.Geometry3D, arg0: numpy.ndarray[float64[4, 4]]) -> None
 |  
 |  ----------------------------------------------------------------------
 |  Methods inherited from Geometry:
 |  
 |  clear(...)
 |      clear(self: open3d.linux.open3d.Geometry) -> None
 |  
 |  dimension(...)
 |      dimension(self: open3d.linux.open3d.Geometry) -> int
 |  
 |  get_geometry_type(...)
 |      get_geometry_type(self: open3d.linux.open3d.Geometry) -> open3d::Geometry::GeometryType
 |  
 |  is_empty(...)
 |      is_empty(self: open3d.linux.open3d.Geometry) -> bool
 |  
 |  ----------------------------------------------------------------------
 |  Data and other attributes inherited from Geometry:
 |  
 |  Image = Type.Image
 |  
 |  LineSet = Type.LineSet
 |  
 |  PointCloud = Type.PointCloud
 |  
 |  TriangleMesh = Type.TriangleMesh
 |  
 |  Type = <class 'open3d.linux.open3d.Type'>
 |  
 |  
 |  Unspecified = Type.Unspecified
 |  
 |  ----------------------------------------------------------------------
 |  Data and other attributes inherited from pybind11_builtins.pybind11_object:
 |  
 |  __new__ = <built-in method __new__ of pybind11_type object>
 |      T.__new__(S, ...) -> a new object with type S, a subtype of T

Help on built-in function read_point_cloud in module open3d.linux.open3d:

read_point_cloud(...)
    read_point_cloud(filename: unicode) -> open3d.linux.open3d.PointCloud
    
    Function to read PointCloud from file

