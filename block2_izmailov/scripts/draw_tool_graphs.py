#!/usr/bin/env python3
import rclpy
import time
import matplotlib.pyplot as plt
from rclpy.node import Node

class FileDataReader(Node):
    def __init__(self):
        super().__init__('file_data_reader')
        self.get_logger().info('Node started. Reading file and plotting data...')
        self.read_and_plot_data()

    def read_and_plot_data(self):
        file_names = ['z', 'y', 'rz']
        dir_name = '/home/student/ros2_ws/block2_izmailov/results/'

        try:
            for file_name in file_names:
                times = []
                values = []
                velocities = []
                accelerations = []
                trh = []

                with open(dir_name + file_name + ".txt", 'r') as file:
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
                            elif data_type == 't':
                                trh.append(value)

                plt.figure()
                plt.plot(times, values)
                plt.xlabel('Cas [s]')
                plt.ylabel('Poloha [rad]')
                plt.grid(True)
                plt.title(f'Poloha {file_name}')

                plt.figure()
                plt.plot(times, velocities)
                plt.xlabel('Cas [s]')
                plt.ylabel('Rychlost [rad / s]')
                plt.grid(True)
                plt.title(f'Rychlost {file_name}')

                plt.figure()
                plt.plot(times, accelerations)
                plt.xlabel('Cas [s]')
                plt.ylabel('Zrychlenie [rad / (s ^ 2)]')
                plt.grid(True)
                plt.title(f'Zrychlenie {file_name}')

                plt.figure()
                plt.plot(times, accelerations)
                plt.xlabel('Cas [s]')
                plt.ylabel('Trh [rad / (s ^ 3)]')
                plt.grid(True)
                plt.title(f'Trh {file_name}')

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
