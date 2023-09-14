import subprocess

def launch_ros2_topic_echo(topic_name, output_file):
    try:
        command = f'ros2 topic echo {topic_name}'
        process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        with open(output_file, 'w') as file:
            while True:
                output = process.stdout.readline()
                if output == '' and process.poll() is not None:
                    break
                if output:
                    print(output.strip())
                    file.write(output)

    except KeyboardInterrupt:
        # Handle user interrupt (Ctrl+C)
        process.terminate()

if __name__ == '__main__':
    topic_name = '/joy'  # Replace with the desired topic name
    output_file = '/home/mistlab/ros2_ws/my_bag2/joy_data.txt'  # Specify the output file name here
    launch_ros2_topic_echo(topic_name, output_file)