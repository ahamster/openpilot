import os

filenames = [
  './selfdrive/ui/qt/offroad/cars/chrysler',
  './selfdrive/ui/qt/offroad/cars/ford',
  './selfdrive/ui/qt/offroad/cars/gm',
  './selfdrive/ui/qt/offroad/cars/honda',
  './selfdrive/ui/qt/offroad/cars/hyundai',
  './selfdrive/ui/qt/offroad/cars/mazda',
  './selfdrive/ui/qt/offroad/cars/nissan',
  './selfdrive/ui/qt/offroad/cars/subaru',
  './selfdrive/ui/qt/offroad/cars/tesla',
  './selfdrive/ui/qt/offroad/cars/toyota',
  './selfdrive/ui/qt/offroad/cars/volkswagen',
]

def generate_cpp_map_code(car_data):
    cpp_code = "// This is an automatically generated file. Run ./launch_chffrplus to generate"
    cpp_code = "std::map<QString, QVector<QString>> vehicleMap = {\n"
    for brand, models in car_data.items():
        models_list = ', '.join(f'"{model}"' for model in models)
        cpp_code += f'    {{"{brand}", {{{models_list}}}}},\n'
    cpp_code += "};\n"
    return cpp_code

def parse_car_files(filenames):
    car_data = {}
    for filename in filenames:
        brand = os.path.basename(filename).capitalize()
        with open(filename, 'r') as file:
            models = [line.strip() for line in file]
        car_data[brand] = models
    return car_data

car_data = parse_car_files(filenames)
cpp_code = generate_cpp_map_code(car_data)

with open('./selfdrive/ui/qt/offroad/cars/allCars.h', 'w') as outfile:
    outfile.write(cpp_code)
