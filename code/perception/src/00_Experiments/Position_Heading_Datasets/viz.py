import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

FILE_START = 15
FILE_END = 33

file_name = "data_26.csv"
folder_path_x = "./x_error/"
folder_path_y = "./y_error/"
file_path_x = folder_path_x + file_name
file_path_y = folder_path_y + file_name
file_path_heading = "./heading_error/" + file_name


# region PLOTS
# DONE
def plot_best_tuned_file_by_type(type='x', error_type='MSE', check_type='IQR'):
    """
    Calculates the best tuned file by type and error type using a specific
    check type.

    Parameters:
    type (str): The type of data to read. Either 'x' or 'y'. (default 'x')
    error_type (str): The type of error to calculate. Either 'MSE', 'MAE' or
                      'RMSPE'. (default 'MSE')
    check_type (str): The type of error to calculate. Either 'IQR' or 'default'

    Returns:

    """
    best_file = ""
    best_val = []
    # best_error_data = ()
    for i in range(FILE_START, FILE_END):
        if i < 10:
            file_name = "data_0" + str(i) + ".csv"
        else:
            file_name = "data_" + str(i) + ".csv"

        ideal, test_filter, current, unfiltered = (
            get_x_or_y_from_csv_file(file_name, type))

        if error_type == 'MSE':
            # Calculate the MSE for each method
            val_test_filter, test_filter_list = calculate_mse_x_or_y(
                                                ideal,
                                                test_filter)
            val_current, current_list = calculate_mse_x_or_y(
                ideal,
                current)
            val_unfiltered, unfiltered_list = calculate_mse_x_or_y(
                ideal,
                unfiltered)
        elif error_type == 'MAE':
            # Calculate the MAE for each method
            val_test_filter, test_filter_list = calculate_mae_x_or_y(
                                                ideal,
                                                test_filter)
            val_current, current_list = calculate_mae_x_or_y(
                ideal,
                current)
            val_unfiltered, unfiltered_list = calculate_mae_x_or_y(ideal,
                                                                   unfiltered)

        vals = [val_test_filter, val_current, val_unfiltered]
        # error_lists = (test_filter_list, current_list, unfiltered_list)

        if check_type == 'IQR':
            q3, q1 = np.percentile(test_filter_list, [80, 0])
            iqr = q3 - q1
            if i == FILE_START:
                best_val = iqr
                best_file = file_name
                # best_error_data = error_lists
            else:
                # compare the vals of the test_filter filter that
                # is tuned the best
                if abs(iqr) < abs(best_val):
                    best_val = iqr
                    best_file = file_name
                    # best_error_data = error_lists
        elif check_type == 'default':
            if i == FILE_START:
                best_val = vals[0]
                best_file = file_name
                # best_error_data = error_lists
            else:
                # compare the vals of the test_filter filter that
                # is tuned the best
                if abs(vals[0]) < abs(best_val):
                    best_val = vals[0]
                    best_file = file_name
                    # best_error_data = error_lists

    # plot the best file
    plot_x_or_y_notched_box(best_file, type=type, error_type=error_type)
    print(best_file)
    print(best_val)


# DONE
def plot_best_tuned_file():
    """
    Plots the best tuned file

    Parameters:
    true_positions (numpy.ndarray): Die tatsächlichen Positionen.
    estimated_positions (numpy.ndarray): Die geschätzten Positionen.

    Returns:
    tuple: best file as a String and the MSE's of the best file.
    """
    best_file = ""
    best_mse = []
    # best_error_data = ()
    for i in range(FILE_START, FILE_END):
        if i < 10:
            file_name = "data_0" + str(i) + ".csv"
        else:
            file_name = "data_" + str(i) + ".csv"

        ideal_pos, test_filter_pos, current_pos, unfiltered_pos = (
            get_positions_from_csv_file(file_name))

        # Calculate the Error for each method by error_type
        val_test_filter, test_filter_list = calculate_mse(ideal_pos,
                                                          test_filter_pos)
        val_current, current_list = calculate_mse(ideal_pos,
                                                  current_pos)
        val_unfiltered, unfiltered_list = calculate_mse(ideal_pos,
                                                        unfiltered_pos)

        mses = [val_test_filter, val_current, val_unfiltered]
        # error_lists = (test_filter_list, current_list, unfiltered_list)

        # Show the plot
        # plt.show()
        if i == 2:
            best_mse = mses
            best_file = file_name
        else:
            # compare the mses of the test_filter filter that is tuned the best
            if mses[0] < best_mse[0]:
                best_mse = mses
                best_file = file_name
                # best_error_data = error_lists

    # plot the best file
    plot_MSE_notched_box(best_file)
    print(best_file)
    print(best_mse)


# DONE
def plot_x_or_y_notched_box(file_name, type='x', error_type='MSE'):
    """
    Calculates and plots the error of x or y data for any given error type.

    Parameters:
    file_name (str): The name of the CSV file.
    type (str): The type of data to read. Either 'x' or 'y'. (default 'x')
    error_type (str): The type of error to calculate.
                      'MSE', 'MAE' (default 'MSE')

    Returns:
    """
    if type == 'x':
        ideal_pos, test_filter_pos, current_pos, unfiltered_pos = (
            get_x_or_y_from_csv_file(file_name, 'x'))
    elif type == 'y':
        ideal_pos, test_filter_pos, current_pos, unfiltered_pos = (
            get_x_or_y_from_csv_file(file_name, 'y'))

    if error_type == 'MSE':
        # Calculate the MSE for each method
        val_test_filter, test_filter_list = calculate_mse_x_or_y(ideal_pos,
                                                            test_filter_pos)
        val_current, current_list = calculate_mse_x_or_y(
            ideal_pos,
            current_pos)
        val_unfiltered, unfiltered_list = calculate_mse_x_or_y(
            ideal_pos,
            unfiltered_pos)
    elif error_type == 'MAE':
        # Calculate the MAE for each method
        val_test_filter, test_filter_list = calculate_mae_x_or_y(ideal_pos,
                                                            test_filter_pos)
        val_current, current_list = calculate_mae_x_or_y(
            ideal_pos,
            current_pos)
        val_unfiltered, unfiltered_list = calculate_mae_x_or_y(ideal_pos,
                                                               unfiltered_pos)

    # Create a new figure
    fig, ax = plt.subplots()

    # Create a list of all positions
    pos_error_list = [test_filter_list, current_list, unfiltered_list]
    # Create a box plot with notches
    boxplot = ax.boxplot(pos_error_list, notch=True,
                         labels=['Test Filter', 'Current', 'Unfiltered'],
                         patch_artist=True)

    # fill with colors and put median vals in the boxes
    colors = ['pink', 'lightblue', 'lightgreen']

    tuple = zip(boxplot['boxes'], colors, boxplot['medians'],
                boxplot['whiskers'][::2])

    for i, (box, color, median) in enumerate(tuple):
        box.set_facecolor(color)
        median_val = median.get_ydata()[1]
        ax.text(i+1, median_val, f'Median: {median_val:.2f}', va='center',
                ha='center', backgroundcolor='white')

        # Calculate IQR
        q3, q1 = np.percentile(pos_error_list[i], [75, 0])
        iqr = q3 - q1

        # Get the y position for the IQR text
        median_y = boxplot['medians'][i].get_ydata()[0]  # height of the notch

        # Add the IQR text
        ax.text(i+0.8, median_y, f'IQR: {iqr:.2f}', va='center',
                ha='center', rotation=90, color='red', backgroundcolor='white')

    # Set the labels
    ax.set_xlabel('Filter')
    ax.set_ylabel(error_type)
    ax.set_title(error_type + ' of ' + type + ' for different methods')
    ax.yaxis.grid(True)

    # Show the plot
    plt.show()


# DONE
def get_positions_from_csv_file(file_name):
    """
    Reads position data from CSV files and returns them as numpy arrays.

    Args:
        file_name (str): The name of the CSV file.

    Returns:
        tuple: A tuple containing four numpy arrays representing the positions.
            - unfiltered_pos: The unfiltered positions.
            - ideal_pos: The ideal positions.
            - test_filter_pos: The positions estimated using test filter.
            - current_pos: The positions estimated using current filter.
    Raises:
        FileNotFoundError: If the specified CSV file does not exist.
    """
    file_path_x = folder_path_x + file_name
    file_path_y = folder_path_y + file_name

    # Read the CSV file into a DataFrame
    df_x = pd.read_csv(file_path_x)
    df_y = pd.read_csv(file_path_y)

    # Set the first column (time) as the index of the DataFrames
    df_x.set_index(df_x.columns[0], inplace=True)
    df_y.set_index(df_y.columns[0], inplace=True)

    # create pos tuples of the x and y data and store them as numpy arrays
    ideal_pos = np.array(list(zip(df_x['Ideal (Carla) X'],
                                  df_y['Ideal (Carla) Y'])))
    test_filter_pos = np.array(list(zip(df_x['Test Filter X'],
                                        df_y['Test Filter Y'])))
    current_pos = np.array(list(zip(df_x['Current X'],
                                    df_y['Current Y'])))
    unfiltered_pos = np.array(list(zip(df_x['Unfiltered X'],
                                       df_y['Unfiltered Y'])))

    return ideal_pos, test_filter_pos, current_pos, unfiltered_pos


# DONE
def get_x_or_y_from_csv_file(file_name, type='x'):
    """
    Reads x or y data from CSV files and returns them as numpy arrays.

    Args:
        file_name (str): The name of the CSV file.
        type (str): The type of data to read. Either 'x' or 'y'. (default 'x')

    Returns:
        tuple: A tuple containing four numpy arrays representing the positions.
            - ideal: The ideal positions.
            - test_filter: The positions estimated using test_filter filtering.
            - current: The positions calculated using a running
              average.
            - unfiltered: The unfiltered positions. """

    if type == 'x':
        file_path = folder_path_x + file_name
    elif type == 'y':
        file_path = folder_path_y + file_name

    # Read the CSV file into a DataFrame
    df = pd.read_csv(file_path, skiprows=8)

    # Set the first column (time) as the index of the DataFrames
    df.set_index(df.columns[0], inplace=True)

    if type == 'x':
        # store x as numpy arrays
        ideal = np.array(df['Ideal (Carla) X'])
        test_filter = np.array(df['Test Filter X'])
        current = np.array(df['Current X'])
        unfiltered = np.array(df['Unfiltered X'])
    elif type == 'y':
        # store y as numpy arrays
        ideal = np.array(df['Ideal (Carla) Y'])
        test_filter = np.array(df['Test Filter Y'])
        current = np.array(df['Current Y'])
        unfiltered = np.array(df['Unfiltered Y'])


    return ideal, test_filter, current, unfiltered


# DONE
def plot_MSE_notched_box(file_name):
    """
    Calculates the Mean Squared Error (MSE) for position data.

    Parameters:
    true_positions (numpy.ndarray): actual positions.
    estimated_positions (numpy.ndarray): estimated positions.

    Returns:
    float: Mean Squared Error (MSE).
    """
    ideal_pos, test_filter_pos, current_pos, unfiltered_pos = (
        get_positions_from_csv_file(file_name))

    # Calculate the MSE for each method
    val_test_filter, test_filter_list = calculate_mse(ideal_pos, test_filter_pos)
    val_current, current_list = calculate_mse(ideal_pos,
                                                      current_pos)
    val_unfiltered, unfiltered_list = calculate_mse(ideal_pos, unfiltered_pos)

    # Create a new figure
    fig, ax = plt.subplots()

    # Create a list of all positions
    pos_mse_list = [test_filter_list, current_list, unfiltered_list]
    # Create a box plot with notches
    boxplot = ax.boxplot(pos_mse_list, notch=True,
                         labels=['Test Filter', 'Current', 'Unfiltered'],
                         patch_artist=True)

    # fill with colors and put median vals in the boxes
    colors = ['pink', 'lightblue', 'lightgreen']
    for i, (box, color, median) in enumerate(zip(boxplot['boxes'], colors,
                                                 boxplot['medians'])):
        box.set_facecolor(color)
        median_val = median.get_ydata()[1]
        ax.text(i+1, median_val, f'Median: {median_val:.2f}', va='center',
                ha='center', backgroundcolor='white')

    # Set the labels
    ax.set_xlabel('Method')
    ax.set_ylabel('MSE')
    ax.set_title('MSE for different methods')
    ax.yaxis.grid(True)

    # Show the plot
    plt.show()


# DONE
def plot_MAE_notched_box(file_name):
    """
    Calculates the Mean Absolute Error (MAE) for position data.

    Parameters:
    true_positions (numpy.ndarray): Actual positions.
    estimated_positions (numpy.ndarray): estimated positions.

    Returns:
    float: Mean Absolute Error (MAE).
    """
    ideal_pos, test_filter_pos, current_pos, unfiltered_pos = (
        get_positions_from_csv_file(file_name))

    # Calculate the MAE for each method
    mae_test_filter, test_filter_list = calculate_mae(ideal_pos,
                                                      test_filter_pos)
    mae_current, current_list = calculate_mae(ideal_pos,
                                              current_pos)
    mae_unfiltered, unfiltered_list = calculate_mae(ideal_pos, unfiltered_pos)

    # Create a new figure
    fig, ax = plt.subplots()

    # Create a list of all positions
    pos_mae_list = [test_filter_list, current_list, unfiltered_list]

    # Create a box plot with notches
    boxplot = ax.boxplot(pos_mae_list, notch=True,
                         labels=['Test Filter', 'Current', 'Unfiltered'],
                         patch_artist=True)

    # fill with colors and put median vals in the boxes
    colors = ['pink', 'lightblue', 'lightgreen']
    for i, (box, color, median) in enumerate(zip(boxplot['boxes'], colors,
                                                 boxplot['medians'])):
        box.set_facecolor(color)
        median_val = median.get_ydata()[1]
        ax.text(i+1, median_val, f'Median: {median_val:.2f}', va='center',
                ha='center', backgroundcolor='white')

    # Set the labels
    ax.set_xlabel('Method')
    ax.set_ylabel('MAE')
    ax.set_title('MAE for different methods')
    ax.yaxis.grid(True)

    # Show the plot
    plt.show()


# DONE
def plot_CEP(file_name):
    """
    Plots the CEP as error circles of different colors in the x-y plane.

    Parameters:
    String: file_name

    Returns:
    """
    file_path_x = folder_path_x + file_name
    file_path_y = folder_path_y + file_name

    # Read the CSV file into a DataFrame
    df_x = pd.read_csv(file_path_x)
    df_y = pd.read_csv(file_path_y)

    # Set the first column (time) as the index of the DataFrames
    df_x.set_index(df_x.columns[0], inplace=True)
    df_y.set_index(df_y.columns[0], inplace=True)

    # create pos tuples of the x and y data and store them as numpy arrays
    ideal_pos = np.array(list(zip(df_x['Ideal (Carla) X'],
                                  df_y['Ideal (Carla) Y'])))
    test_filter_pos = np.array(list(zip(df_x['Test Filter X'],
                                        df_y['Test Filter Y'])))
    current_pos = np.array(list(zip(df_x['Current X'],
                                    df_y['Current Y'])))
    unfiltered_pos = np.array(list(zip(df_x['Unfiltered X'],
                                       df_y['Unfiltered Y'])))

    # create CEP for each method
    cep_test_filter, cep_current, cep_unfiltered = calculate_cep(
        ideal_pos, test_filter_pos, current_pos, unfiltered_pos)

    # plot the cep as error circles of different colors in the x-y plane
    # Create a new figure
    fig, ax = plt.subplots()

    # Create circles with the given radii
    circle_test_filter = plt.Circle((0, 0), cep_test_filter, fill=False,
                               label='Test Filter',
                               color='r')
    circle_current = plt.Circle((0, 0), cep_current, fill=False,
                                    label='Current', color='g')
    circle_unfiltered = plt.Circle((0, 0), cep_unfiltered, fill=False,
                                   label='Unfiltered', color='b')

    # Add the circles to the plot
    ax.add_artist(circle_test_filter)
    ax.add_artist(circle_current)
    ax.add_artist(circle_unfiltered)

    # Set the limits of the plot to show all circles
    ax.set_xlim(-max(cep_test_filter, cep_current, cep_unfiltered),
                max(cep_test_filter, cep_current, cep_unfiltered))
    ax.set_ylim(-max(cep_test_filter, cep_current, cep_unfiltered),
                max(cep_test_filter, cep_current, cep_unfiltered))

    # Add a legend
    plt.legend()

    # Add a grid
    plt.grid(True)

    # Set the y-axis label to 'Distance in Meters'
    plt.ylabel('Distance in Meters')

    # Set the x-axis label to 'Distance in Meters'
    plt.xlabel('Distance in Meters')


# DONE
def plot_csv_heading2(file_path):
    # Read the CSV file into a DataFrame
    df = pd.read_csv(file_path)

    # Set the first column (time) as the index of the DataFrame
    df.set_index(df.columns[0], inplace=True)

    # Select the first three columns
    df_to_plot = df[df.columns[:3]]

    # Plot the 'test_filter_heading' and 'current_heading' columns with default
    # line style
    plt.plot(df_to_plot['Test Filter'], label='Test Filter Heading')
    plt.plot(df_to_plot['Current'], label='Current Heading')

    # Plot the 'ideal_heading' column with a blue dotted line
    plt.plot(df_to_plot['Ideal (Carla)'], 'r:', label='Ideal Heading')
    # Display the legend
    plt.legend()
    # Plot the DataFrame
    # df_to_plot.plot()

    # Add a grid
    plt.grid(True)

    # Set the y-axis label to 'Radians'
    plt.ylabel('Heading in Radians')

    # Set the x-axis label to 'Time'
    plt.xlabel('Time in seconds')
# endregion PLOTS

# DONE
# region CALCUATIONS
def calculate_mae(ideal, estimated):
    """
    Calculates the Mean Absolute Error (MAE) for position data.

    Parameters:
    ideal (numpy.ndarray): The ideal positions.
    estimated (numpy.ndarray): The estimated positions.

    Returns:
    Tuple: A tuple containing the MAE and the error for each position.
    """
    # Calculate the errors
    error = np.linalg.norm(ideal - estimated, axis=1)

    # Calculate the MAE
    mae = np.mean(error)

    return mae, error


def calculate_mae_x_or_y(ideal, estimated):
    """
    Calculates the Mean Absolute Error (MAE) for x or y data.

    Parameters:
    ideal (numpy.ndarray): The ideal x or y positions.
    estimated (numpy.ndarray): The estimated x or y positions.

    Returns:
    Tuple: A tuple containing the MAE and the error for each position.
    """
    # Calculate the errors
    error = np.abs(ideal - estimated)

    # Calculate the MAE
    mae = np.mean(error)

    return mae, error


def calculate_mse(ideal, estimated):
    """
    Calculates the Mean Squared Error (MSE) for position data.

    Parameters:
    ideal (numpy.ndarray): The ideal positions.
    estimated (numpy.ndarray): The estimated positions.

    Returns:
    Tuple: A tuple containing the MSE and the error for each position.
    """
    # Calculate the errors
    error = np.linalg.norm(ideal - estimated, axis=1)**2

    # Calculate the MSE
    mse = np.mean(error)

    return mse, error


def calculate_mse_x_or_y(ideal, estimated):
    """
    Calculates the Mean Squared Error (MSE) for x or y data.

    Parameters:
    ideal (numpy.ndarray): The ideal x or y positions.
    estimated (numpy.ndarray): The estimated x or y positions.

    Returns:
    Tuple: A tuple containing the MSE and the error for each position.
    """
    # Calculate the errors
    error = (ideal - estimated)**2

    # Calculate the MSE
    mse = np.mean(error)

    return mse, error


def calculate_cep(ideal, test_filter, current, unfiltered, percentile=90):
    """
    Calculates the Circular Error Probable (CEP) for position data.

    Parameters:
    ideal (numpy.ndarray): The ideal positions.
    test_filter (numpy.ndarray): The positions estimated using test_filter
                                 filtering.
    current (numpy.ndarray): The positions calculated using the current
                             filter.
    unfiltered (numpy.ndarray): The unfiltered positions.
    percentile (int): The percentile to use when calculating the CEP.

    Returns:
    tuple: A tuple containing the CEP for each method.
    """
    # Calculate the errors
    error_test_filter = np.sqrt(np.sum((test_filter - ideal)**2, axis=1))
    error_current = np.sqrt(np.sum((current - ideal)**2, axis=1))
    error_unfiltered = np.sqrt(np.sum((unfiltered - ideal)**2, axis=1))

    # Calculate the CEP for each method
    cep_test_filter = np.percentile(error_test_filter, percentile)
    cep_current = np.percentile(error_current, percentile)
    cep_unfiltered = np.percentile(error_unfiltered, percentile)

    return cep_test_filter, cep_current, cep_unfiltered

# endregion CALCUATIONS


# # plot_RMSPE_notched_box("data_26.csv")
# plot_CEP("data_25.csv")
# plot_CEP("data_26.csv")
data = "data_50.csv"
plot_CEP(data)
plot_x_or_y_notched_box(data, type='x', error_type='MSE')
plot_x_or_y_notched_box(data, type='x', error_type='MAE')
# plot_x_or_y_notched_box(data, type='y', error_type='MSE')
# plot_x_or_y_notched_box(data, type='y', error_type='MAE')

plt.show()
