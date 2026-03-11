import os

MODEL_URI_PREFIX = 'model://'

sdf_paths = []
if os.getenv("IGN_GAZEBO_RESOURCE_PATH") is not None:
    sdf_paths = sdf_paths+os.getenv("IGN_GAZEBO_RESOURCE_PATH").split(":")
if os.getenv("GAZEBO_MODEL_PATH") is not None:
    sdf_paths = sdf_paths+os.getenv("GAZEBO_MODEL_PATH").split(":")

def get_model_directory(model_name):
    for tmp_dir in sdf_paths:
        model_directory = os.path.join(tmp_dir, model_name)
        if(os.path.isdir(model_directory)):
            return model_directory
    return ""

# get absolute path according to uri
def parse_model_uri(uri):
    if uri.find(MODEL_URI_PREFIX) != 0:
        return ''
    tmp_uri = uri[len(MODEL_URI_PREFIX):]
    pos = tmp_uri.find('/')
    if pos == -1:
        return ''
    model_name = tmp_uri[0:pos]
    model_dir_path = get_model_directory(model_name)
    if model_dir_path == '':
        return ''
    return model_dir_path + tmp_uri[pos:]
