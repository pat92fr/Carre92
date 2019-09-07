import my_parameters as params
import my_datasettools as dtools

## MAIN ########################################################################

# load dataset file
X,Y = dtools.load_dataset(params.dataset_dir,  params.dataset_filename, display = True)

