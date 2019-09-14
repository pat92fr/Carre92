import my_parameters as params
import my_datasettools as dtools

## MAIN ########################################################################

for directory in params.dataset_dir:
    print(directory)
    # load dataset file
    dtools.load_dataset(directory,  params.dataset_filename, display = True)

