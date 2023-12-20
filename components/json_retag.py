import os
import json 

def json_retag():
    workdir = os.path.dirname(os.path.abspath(__file__))
    repos = os.listdir()
    repos.remove('docker_retag.py')
    repos.remove('generate_readme.py')
    repos.remove('generate_table.py')
    repos.remove('json_retag.py')
    repos.remove('README.md')
    repos.remove('.filter_only_updated_items.py')
    repos.remove('.gitignore')
    repos.remove('.gitlab-ci.yml')
    repos.remove('.git')
    
    # repos.remove('caffe-object-detection')
    repos.remove('isaac-skeleton-viewer')
    # repos.remove('slam-toolbox')
    repos.remove('hamster-v8-environment')

    for repo in repos:
        print(repo)
        repo_path = os.path.join(workdir, repo)
        nimbus_dir = os.listdir(repo_path)
        
        files_to_remove = ['docker', 'README.md', '.catkin_workspace']
        
        for file_name in files_to_remove:
            try:
                nimbus_dir.remove(file_name)
            except ValueError:
                pass

        for dir in nimbus_dir:
            json_file_path = os.path.join(repo_path, dir, 'nimbusc.json')

            with open(json_file_path, 'r') as json_file:
                json_content = json.load(json_file)

            docker_image = json_content['environment']['dockerInfo']['image'].split(':')[0] # Extract the docker image name from the json file   
            json_content['environment']['dockerInfo']['image'] = f"{docker_image}:melodic"

            with open(f'{json_file_path}', 'w') as json_file:
                json.dump(json_content, json_file, indent=4)


json_retag()