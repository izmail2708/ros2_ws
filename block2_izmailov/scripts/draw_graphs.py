#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt

class FileDataReader(Node):
    def __init__(self):
        super().__init__('file_data_reader')
        self.get_logger().info('Node started. Reading file and plotting data...')
        self.read_and_plot_data()

    def read_and_plot_data(self):
        file_names = ['klb_0.txt', 'klb_2.txt']
        dir_name = '/home/student/ros2_ws/block2_izmailov/results/'

        try:
            for file_name in file_names:
                times = []
                values = []
                velocities = []
                accelerations = []

                with open(dir_name + file_name, 'r') as file:
                    for line in file:
                        parts = line.strip().split(':')
                        if len(parts) == 3:
                            data_type = parts[0]
                            time = float(parts[1])
                            value = float(parts[2])

                            if data_type == 'p':
                                times.append(time)
                                values.append(value)
                            elif data_type == 'v':
                                velocities.append(value)
                            elif data_type == 'a':
                                accelerations.append(value)

                plt.figure()
                plt.plot(times, values, label='q')
                plt.plot(times, velocities, label='q^1')
                plt.plot(times, accelerations, label='q^2')
                plt.xlabel('Cas (s)')
                plt.ylabel('Hodnota')
                plt.title(f'{file_name}')
                plt.legend()
                plt.grid(True)

            plt.show()

        except Exception as e:
            self.get_logger().error(f'Error reading file: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = FileDataReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
