import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from obstacles_msg.msg import ObstaclesArray, Obstacles


class ObstaclesManager(Node):

    def __init__(self):

        super().__init__('obstacles_manager')
        
        self.declare_parameter('robot_ids', [''])

        self.robot_ids = self.get_parameter('robot_ids').get_parameter_value().string_array_value

        self.declare_parameter('static_obs_list', [0.0, 0.0, 0.0]) ##append at the back, pop the index correspond to robot_id

        self.static_obs_list = self.get_parameter('static_obs_list').get_parameter_value().double_array_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.05, self.update_obstacles)

        self.publisher = self.create_publisher(ObstaclesArray, 'obstacles', 10)

    def update_obstacles(self):

        source_frame = 'map'
        obs_list = ObstaclesArray()

        # for id in self.robot_ids:
        #     target_frame = 'robot_'+id

        #     try:
        #         t = self.tf_buffer.lookup_transform(
        #             target_frame,
        #             source_frame,
        #             rclpy.time.Time())
        #     except TransformException as ex:
        #         # self.get_logger().info(
        #         #     f'Could not transform {source_frame} to {target_frame}: {ex}')
                
        #         return

        #     obs = Obstacles()
        #     obs.name = id
        #     obs.x = t.transform.translation.x
        #     obs.y = t.transform.translation.y
        #     obs.prohibited_rad = 0.25

        #     obs_list.obstacles_array[int(id)] = obs


        obs = Obstacles()
        obs.name = 'static_obs_'+str(self.static_obs_list[0])+'_'+str(self.static_obs_list[1])
        obs.x = self.static_obs_list[0]
        obs.y = self.static_obs_list[1]
        obs.prohibited_rad = self.static_obs_list[2]
        obs_list.obstacles_array.append(obs)

        self.publisher.publish(obs_list)


def main(args=None):
    rclpy.init(args=args)
    manager=ObstaclesManager()
    print("Starting...\n")
    rclpy.spin(manager) 
    rclpy.shutdown()
    print("Finished\n")

if __name__ == '__main__':
    #initalize the processes
    main()

        






    