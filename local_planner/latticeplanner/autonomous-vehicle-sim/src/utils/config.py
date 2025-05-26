def load_config(file_path='config.yaml'):
    import yaml

    with open(file_path, 'r') as file:
        config = yaml.safe_load(file)

    return config

def get_vehicle_params(config):
    return config.get('vehicle', {})

def get_map_settings(config):
    return config.get('map', {})