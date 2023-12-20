import subprocess
import json
import os

# Get the location of the script, script should work from the library directory
DIR_PATH = script_directory = os.path.dirname(os.path.abspath(__file__))

# All of the repos at the dictionary have multiple nimbus components based on the same image. The dictionary will return the relevant docker
# image based on the repo name in order to avoid multiple retags for the same image.
images_reamp = {'custom-message-talker-listener':'cognimbus/custome-message-tutorial', 'custom-ros-service':'cognimbus/custom-ros-service',
                'hokuyo':'cognimbus/hokuyo', 'kobuki-driver':'cognimbus/kobuki_driver', 'lynx':'cognimbus/lynx',
                'openvino':'intelpengo/openvino', 'realsense-camera':'cognimbus/realsense2', 'ros_audio':'cognimbus/ros-audio',
                'slamtec-rplidar-driver':'cognimbus/rplidar-ros', 'turtlebot3':'cognimbus/turtlebot3-wafflepi',
                'ubiquity-driver':'cognimbus/ubiquity-driver'}

# The method gets a shell command and execute it
def run_command(command):
    """
    Run a shell command and return the output as a string.
    """
    process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    output, error = process.communicate()
    if process.returncode != 0:
        raise RuntimeError(f"Command '{command}' failed with error: {error.decode()}")
    return output.decode()

# This method extracts the ros version from the repo's docker file
def get_ros_version(repo):
    path = os.path.join(DIR_PATH, repo, 'docker', 'Dockerfile')  # Get the full path for the docker file
    version = open(path, 'r').readline().split(':')[-1]          # first line of docker file: "FROM ros:<ros version>"
    if '-' in version:                                           
        version = version.split('-')[0]
    if '\n' in version:
        version = version.split('\n')[0]                         # Remove \n symbol if exists
    return version

# This method extracts the docker image name that nimbus is using from the component's json file
def get_docker_image(repo):
    if repo in images_reamp:
        # Return the docker image from the dictionary for repositories contains multiple coponents based on the same docker image
        return images_reamp.get(repo)
    
    path = os.path.join(DIR_PATH, repo, repo, 'nimbusc.json')       # Get the full path for the nimbus json file
    with open(path, 'r') as file:
        data = json.load(file)
    image = data['environment']['dockerInfo']['image']              # Extract the docker image name from the json file

    if ':' in image:
        return image.split(':')[0]                                  # Remove the image's tag if exists
    return image

# This method retag a docker manifest with current tag to new_tag
def retag_docker_manifest(image, tag, new_tag):
    # Step 1: Get the images in the manifest
    inspect_command = f"docker manifest inspect {image}:{tag}"
    inspect_output = run_command(inspect_command)
    output_json = json.loads(inspect_output)
    
    # Step 2: Extract the digest of the each image in the manifest
    image_digests = [image['digest'] for image in output_json['manifests']]

    # Step 2: Create a new manifest with the image digests
    amend_commands = " ".join([f"--amend {image}@{image_digest}" for image_digest in image_digests])
    create_command = f"docker manifest create {image}:{new_tag} {amend_commands}"
    
    create_output = run_command(create_command)

    # Step 3: Push the new manifest
    push_command = f"docker manifest push {image}:{new_tag}"
    run_command(push_command)

def run_script():
    repos = os.listdir(DIR_PATH)
    # Remove unrelevant/problematic folders names from the library folder
    repos.remove('unclassified')
    repos.remove('generate_readme.py')
    repos.remove('README.md')
    repos.remove('.git')
    repos.remove('.gitignore')
    repos.remove('.gitlab-ci.yml')
    repos.remove('generate_table.py')

    repos.remove('.filter_only_updated_items.py')  # Unknown
    repos.remove('hamster-v8-environment')         # hamster environment should not be in this repo, driver only
    repos.remove('isaac-skeleton-viewer')          # Do not contain docker file and docker image
    repos.remove('slam-toolbox')                   # No "parameters" section, ROS2

    # Remove all the components that dont support multi architecture (these components have no docker manifest)
    repos.remove('arducam-jetson')                      #arm64
    repos.remove('jetson-isaac-skeleton')               #arm64
    repos.remove('lizi-driver')                         #amd64
    repos.remove('mir-driver')                          #amd64
    repos.remove('omron-ld60-driver')                   #amd64
    repos.remove('openvino')                            #amd64
    repos.remove('ros-deep-learning-jetson-inference')  #arm64
    repos.remove('ros1-gateway-arm32')                  #arm32
    repos.remove('sweep-lidar')                         #amd64
    repos.remove('turtlebot3')                          #arm64

    for repo in sorted(repos):
        print(repo)
        ros_version_tag = get_ros_version(repo)
        docker_image = get_docker_image(repo)
        retag_docker_manifest(docker_image, 'latest', ros_version_tag)
        ##TODO: push a1 manifest manualy

# run_script()