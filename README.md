#### Step 1: Extended Kalman Filter to track a single real-world target with lidar measurement input over time

setting:

```

training_segment-10072231702153043603_5725_000_5745_000_with_camera_labels.tfrecord

show_only_frames = [150, 200]
configs_det = det.load_configs(model_name='fpn_resnet')


configs_det.lim_y = [-5, 10]

exec_detection = []

exec_tracking = ['perform_tracking']

exec_visualization = ['show_tracks']

```

results:
For tracking a single vehicle, I got a RSMSE score of 0.31 !
![](/img/fig1.png)
![](/img/Figure_2.png)

<br/>

#### Step 2: Track management to initialize and delete tracks

Manage initialization of new detected vehicles, updating tracks, and deleting them.

setting:

```

training_segment-10072231702153043603_5725_000_5745_000_with_camera_labels.tfrecord

show_only_frames = [65, 100]
configs_det = det.load_configs(model_name='fpn_resnet')


configs_det.lim_y = [-5, 15]

exec_detection = []

exec_tracking = ['perform_tracking']

exec_visualization = ['show_tracks']

```

results:
![](/img/2-1.png)
![](/img/2-2.png)
![](/img/2-3.png)

<br/>

#### Step 3: a single nearest neighbor data association to associate measurements to tracks

Assign new measurements to current tracks using simple nearest neighbor to have multi-target tracking.
setting:

```

training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord

show_only_frames = [0, 200]
configs_det = det.load_configs(model_name='fpn_resnet')


configs_det.lim_y = [-5, 15]

exec_detection = []

exec_tracking = ['perform_tracking']

exec_visualization = ['show_tracks']

```

results:
![](/img/3.png)
![](/img/3-2.png)
<br/>

#### Step 4: Camera sensor fusion

Implement camera model and perform multi-target tracking.

setting:

```

training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord

show_only_frames = [0, 200]
configs_det = det.load_configs(model_name='fpn_resnet')


configs_det.lim_y = [-5, 15]

exec_detection = []

exec_tracking = ['perform_tracking']

exec_visualization = ['show_tracks']

```

results:
![](/img//4_1.png)
![](/img/4_2.png)

# Questions

##### Do you see any benefits in camera-lidar fusion compared to lidar-only tracking (in theory and in your concrete results)?

Fusing data from different sensors has many advantages, it overcome the short comingin Lidar (can't work certain weather conditions, can only measure distance and velocity) and Camera (can be used to detect object but it cannot give an accurate distance estimation). In this project, we can compare results from Step 2 and Step 4, the latter uses both Lidar and Camera whcih results in lower RMSE than the former that only used Lidar.

##### Which challenges will a sensor fusion system face in real-life scenarios? Did you see any of these challenges in the project?

Using multiple sensors add a layer of comlixty to the project and it increases the processing time. Also, in some cases, object is detected by one sensor and not other.

##### Can you think of ways to improve your tracking results in the future?

In project 1, we used to SSD model to detect object, so I think if we use it here it wil help improve the performance.
