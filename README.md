# ADAS hyperparameter tuning with GA

A project for hyperparameter selection with usage of Genetic Algorithm.
All data in point cloud format is taken from[A2D2 dataset](https://www.a2d2.audi/a2d2/en.html).
An aim of the solution is to automatize and optimize hyperparameter tuning of various classification algorithms, which is done manually nowadays.
Genetic Algorithm was chosen for the project as a new and perspective way for optimization without ground truth data.
[EasyGA](https://github.com/danielwilczak101/EasyGA) library is used for Genetic algorithm implementation,
fitness function is based on existing [clustering metrics](https://scikit-learn.org/stable/modules/clustering.html)

# Example of usage
```python
Chooses best hyperparameters

positional arguments:
lidar_path          Directory where lidar files are stored
images_path         Directory where images are stored
start_indx          From which file algorithm should start
num_of_shots        How many files are in the sequence
default             Use default parameters or not
generation_goal     Number of generations
population_size     Number of chromosomes in generation
{1,2,3}             Silhouette = 1, Calinski-Harabasz Index = 2, Calinski-Harabasz Index = 3
{1,2,3,4}           point clouds = 1, point clouds with between-clusters distances = 2,  mapped images = 3, points selection mode = 4
verbose             print side info or not

```