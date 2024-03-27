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
def plot_best_tuned_file_by_type(type='x', error_type='MSE', check_type='IQR'):
    """
    Berechnet den Error Typ für x oder y Daten.

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

        ideal, kalman, running_avg, unfiltered = (
            get_x_or_y_from_csv_file(file_name, type))

        if error_type == 'MSE':
            # Calculate the MSE for each method
            val_kalman, kalman_list = calculate_mse_x_or_y(ideal, kalman)
            val_running_avg, running_avg_list = calculate_mse_x_or_y(
                ideal,
                running_avg)
            val_unfiltered, unfiltered_list = calculate_mse_x_or_y(
                ideal,
                unfiltered)
        elif error_type == 'MAE':
            # Calculate the MAE for each method
            val_kalman, kalman_list = calculate_mae_x_or_y(ideal, kalman)
            val_running_avg, running_avg_list = calculate_mae_x_or_y(
                ideal,
                running_avg)
            val_unfiltered, unfiltered_list = calculate_mae_x_or_y(ideal,
                                                                   unfiltered)
        elif error_type == 'RMSPE':
            # Calculate the RMSPE for each method
            val_kalman, kalman_list = calculate_RMSPE_x_or_y(ideal,
                                                             kalman)
            val_running_avg, running_avg_list = calculate_RMSPE_x_or_y(
                ideal, running_avg)
            val_unfiltered, unfiltered_list = calculate_RMSPE_x_or_y(
                ideal, unfiltered)

        vals = [val_kalman, val_running_avg, val_unfiltered]
        # error_lists = (kalman_list, running_avg_list, unfiltered_list)

        if check_type == 'IQR':
            q3, q1 = np.percentile(kalman_list, [80, 0])
            iqr = q3 - q1
            if i == FILE_START:
                best_val = iqr
                best_file = file_name
                # best_error_data = error_lists
            else:
                # compare the vals of the kalman filter that is tuned the best
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
                # compare the vals of the kalman filter that is tuned the best
                if abs(vals[0]) < abs(best_val):
                    best_val = vals[0]
                    best_file = file_name
                    # best_error_data = error_lists

    # plot the best file
    plot_x_or_y_notched_box(best_file, type=type, error_type=error_type)
    print(best_file)
    print(best_val)


def plot_best_tuned_file():
    """
    Berechnet den Mean Squared Error (MSE) für Positionsdaten.

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

        ideal_pos, kalman_pos, running_avg_pos, unfiltered_pos = (
            get_positions_from_csv_file(file_name))

        # Calculate the Error for each method by error_type
        val_kalman, kalman_list = calculate_mse(ideal_pos, kalman_pos)
        val_running_avg, running_avg_list = calculate_mse(ideal_pos,
                                                          running_avg_pos)
        val_unfiltered, unfiltered_list = calculate_mse(ideal_pos,
                                                        unfiltered_pos)

        mses = [val_kalman, val_running_avg, val_unfiltered]
        # error_lists = (kalman_list, running_avg_list, unfiltered_list)

        # Show the plot
        # plt.show()
        if i == 2:
            best_mse = mses
            best_file = file_name
        else:
            # compare the mses of the kalman filter that is tuned the best
            if mses[0] < best_mse[0]:
                best_mse = mses
                best_file = file_name
                # best_error_data = error_lists

    # plot the best file
    plot_MSE_notched_box(best_file)
    print(best_file)
    print(best_mse)


def plot_x_or_y_notched_box(file_name, type='x', error_type='MSE'):
    """
    Berechnet den Mean Squared Error (MSE) für Positionsdaten.

    Parameters:
    file_name (str): The name of the CSV file.
    type (str): The type of data to read. Either 'x' or 'y'. (default 'x')
    error_type (str): The type of error to calculate. Either 'MSE', 'MAE' or
                      'RMSPE'. (default 'MSE')

    Returns:
    """
    if type == 'x':
        ideal_pos, kalman_pos, running_avg_pos, unfiltered_pos = (
            get_x_or_y_from_csv_file(file_name, 'x'))
    elif type == 'y':
        ideal_pos, kalman_pos, running_avg_pos, unfiltered_pos = (
            get_x_or_y_from_csv_file(file_name, 'y'))

    if error_type == 'MSE':
        # Calculate the MSE for each method
        val_kalman, kalman_list = calculate_mse_x_or_y(ideal_pos, kalman_pos)
        val_running_avg, running_avg_list = calculate_mse_x_or_y(
            ideal_pos,
            running_avg_pos)
        val_unfiltered, unfiltered_list = calculate_mse_x_or_y(
            ideal_pos,
            unfiltered_pos)
    elif error_type == 'MAE':
        # Calculate the MAE for each method
        val_kalman, kalman_list = calculate_mae_x_or_y(ideal_pos, kalman_pos)
        val_running_avg, running_avg_list = calculate_mae_x_or_y(
            ideal_pos,
            running_avg_pos)
        val_unfiltered, unfiltered_list = calculate_mae_x_or_y(ideal_pos,
                                                               unfiltered_pos)
    elif error_type == 'RMSPE':
        # Calculate the RMSPE for each method
        val_kalman, kalman_list = calculate_RMSPE_x_or_y(
            ideal_pos,
            kalman_pos)
        val_running_avg, running_avg_list = calculate_RMSPE_x_or_y(
            ideal_pos, running_avg_pos)
        val_unfiltered, unfiltered_list = calculate_RMSPE_x_or_y(
            ideal_pos, unfiltered_pos)

    # Create a new figure
    fig, ax = plt.subplots()

    # Create a list of all positions
    pos_error_list = [kalman_list, running_avg_list, unfiltered_list]
    # Create a box plot with notches
    boxplot = ax.boxplot(pos_error_list, notch=True,
                         labels=['Kalman', 'Running Avg', 'Unfiltered'],
                         patch_artist=True)

    # fill with colors and put median vals in the boxes
    colors = ['pink', 'lightblue', 'lightgreen']

    tuple = zip(boxplot['boxes'], colors, boxplot['medians'],
                boxplot['whiskers'][::2])

    for i, (box, color, median, whisker) in enumerate(tuple):
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


def get_positions_from_csv_file(file_name):
    """
    Reads position data from CSV files and returns them as numpy arrays.

    Args:
        file_name (str): The name of the CSV file.

    Returns:
        tuple: A tuple containing four numpy arrays representing the positions.
            - ideal_pos: The ideal positions.
            - kalman_pos: The positions estimated using Kalman filtering.
            - running_avg_pos: The positions calculated using a running
              average.
            - unfiltered_pos: The unfiltered positions.

    Raises:
        FileNotFoundError: If the specified CSV file does not exist.
    """
    file_path_x = folder_path_x + file_name
    file_path_y = folder_path_y + file_name

    # Read the CSV file into a DataFrame
    df_x = pd.read_csv(file_path_x)
    df_y = pd.read_csv(file_path_y)

    # rename the columns
    df_x.columns = ['time', 'ideal_x', 'kalman_x', 'running_avg_x',
                    'unfiltered_x', 'kalman_debug_x', 'current_debug_x',
                    'unfiltered_debug_x']
    df_y.columns = ['time', 'ideal_y', 'kalman_y', 'running_avg_y',
                    'unfiltered_y', 'kalman_debug_y', 'current_debug_y',
                    'unfiltered_debug_y']

    # Set the first column (time) as the index of the DataFrames
    df_x.set_index(df_x.columns[0], inplace=True)
    df_y.set_index(df_y.columns[0], inplace=True)

    # create pos tuples of the x and y data and store them as numpy arrays
    ideal_pos = np.array(list(zip(df_x['ideal_x'], df_y['ideal_y'])))
    kalman_pos = np.array(list(zip(df_x['kalman_x'], df_y['kalman_y'])))
    running_avg_pos = np.array(list(zip(df_x['running_avg_x'],
                                        df_y['running_avg_y'])))
    unfiltered_pos = np.array(list(zip(df_x['unfiltered_x'],
                                       df_y['unfiltered_y'])))

    return ideal_pos, kalman_pos, running_avg_pos, unfiltered_pos


def get_x_or_y_from_csv_file(file_name, type='x'):
    """
    Reads x or y data from CSV files and returns them as numpy arrays.

    Args:
        file_name (str): The name of the CSV file.
        type (str): The type of data to read. Either 'x' or 'y'. (default 'x')

    Returns:
        tuple: A tuple containing four numpy arrays representing the positions.
            - ideal: The ideal positions.
            - kalman: The positions estimated using Kalman filtering.
            - running_avg: The positions calculated using a running
              average.
            - unfiltered: The unfiltered positions. """

    if type == 'x':
        file_path = folder_path_x + file_name
    elif type == 'y':
        file_path = folder_path_y + file_name

    # Read the CSV file into a DataFrame
    df = pd.read_csv(file_path, skiprows=8)

    # rename the columns
    df.columns = ['time', 'ideal_x', 'kalman_x', 'running_avg_x',
                  'unfiltered_x', 'kalman_debug_x', 'current_debug_x',
                  'unfiltered_debug_x']

    # Set the first column (time) as the index of the DataFrames
    df.set_index(df.columns[0], inplace=True)

    # store x as numpy arrays
    ideal = np.array(df['ideal_x'])
    kalman = np.array(df['kalman_x'])
    running_avg = np.array(df['running_avg_x'])
    unfiltered = np.array(df['unfiltered_x'])

    return ideal, kalman, running_avg, unfiltered


def plot_RMSPE_notched_box(file_name):
    """
    Berechnet den Root Mean Squared Percentage Error (RMSPE)
    für Positionsdaten.

    Parameters:
    String: file_name

    Returns:
    """

    ideal_pos, kalman_pos, running_avg_pos, unfiltered_pos = (
        get_positions_from_csv_file(file_name))

    # Calculate the RMSPE for each method
    rmspe_kalman, kalman_list = calculate_RMSPE(ideal_pos, kalman_pos)
    rmspe_running_avg, running_avg_list = calculate_RMSPE(ideal_pos,
                                                          running_avg_pos)
    rmspe_unfiltered, unfiltered_list = calculate_RMSPE(ideal_pos,
                                                        unfiltered_pos)

    # Create a new figure
    fig, ax = plt.subplots()

    # Create a list of all positions
    pos_rmspe_list = [kalman_list, running_avg_list, unfiltered_list]
    # Create a box plot with notches
    boxplot = ax.boxplot(pos_rmspe_list, notch=True,
                         labels=['Kalman', 'Running Avg', 'Unfiltered'],
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
    ax.set_ylabel('RMSPE')
    ax.set_title('RMSPE for different methods')
    ax.yaxis.grid(True)

    # Show the plot
    plt.show()


def plot_MSE_notched_box(file_name):
    """
    Berechnet den Mean Squared Error (MSE) für Positionsdaten.

    Parameters:
    true_positions (numpy.ndarray): Die tatsächlichen Positionen.
    estimated_positions (numpy.ndarray): Die geschätzten Positionen.

    Returns:
    float: Der Mean Squared Error (MSE).
    """
    ideal_pos, kalman_pos, running_avg_pos, unfiltered_pos = (
        get_positions_from_csv_file(file_name))

    # Calculate the MSE for each method
    val_kalman, kalman_list = calculate_mse(ideal_pos, kalman_pos)
    val_running_avg, running_avg_list = calculate_mse(ideal_pos,
                                                      running_avg_pos)
    val_unfiltered, unfiltered_list = calculate_mse(ideal_pos, unfiltered_pos)

    # Create a new figure
    fig, ax = plt.subplots()

    # Create a list of all positions
    pos_mse_list = [kalman_list, running_avg_list, unfiltered_list]
    # Create a box plot with notches
    boxplot = ax.boxplot(pos_mse_list, notch=True,
                         labels=['Kalman', 'Running Avg', 'Unfiltered'],
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


def plot_MAE_notched_box(file_name):
    """
    Berechnet den Mean Absolute Error (MAE) für Positionsdaten.

    Parameters:
    true_positions (numpy.ndarray): Die tatsächlichen Positionen.
    estimated_positions (numpy.ndarray): Die geschätzten Positionen.

    Returns:
    float: Der Mean Absolute Error (MAE).
    """
    ideal_pos, kalman_pos, running_avg_pos, unfiltered_pos = (
        get_positions_from_csv_file(file_name))

    # Calculate the MAE for each method
    mae_kalman, kalman_list = calculate_mae(ideal_pos, kalman_pos)
    mae_running_avg, running_avg_list = calculate_mae(ideal_pos,
                                                      running_avg_pos)
    mae_unfiltered, unfiltered_list = calculate_mae(ideal_pos, unfiltered_pos)

    # Create a new figure
    fig, ax = plt.subplots()

    # Create a list of all positions
    pos_mae_list = [kalman_list, running_avg_list, unfiltered_list]

    # Create a box plot with notches
    boxplot = ax.boxplot(pos_mae_list, notch=True,
                         labels=['Kalman', 'Running Avg', 'Unfiltered'],
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


def plot_CEP(file_name):
    """
    Berechnet den Circular Error Probable (CEP) für Positionsdaten.

    Parameters:
    String: file_name

    Returns:
    """
    file_path_x = folder_path_x + file_name
    file_path_y = folder_path_y + file_name

    # Read the CSV file into a DataFrame
    df_x = pd.read_csv(file_path_x)
    df_y = pd.read_csv(file_path_y)

    # rename the columns
    df_x.columns = ['time', 'ideal_x', 'kalman_x', 'running_avg_x',
                    'unfiltered_x', 'kalman_debug_x', 'current_debug_x',
                    'unfiltered_debug_x']
    df_y.columns = ['time', 'ideal_y', 'kalman_y', 'running_avg_y',
                    'unfiltered_y', 'kalman_debug_y', 'current_debug_y',
                    'unfiltered_debug_y']

    # Set the first column (time) as the index of the DataFrames
    df_x.set_index(df_x.columns[0], inplace=True)
    df_y.set_index(df_y.columns[0], inplace=True)

    # create pos tuples of the x and y data and store them as numpy arrays
    ideal_pos = np.array(list(zip(df_x['ideal_x'], df_y['ideal_y'])))
    kalman_pos = np.array(list(zip(df_x['kalman_x'], df_y['kalman_y'])))
    running_avg_pos = np.array(list(zip(df_x['running_avg_x'],
                                        df_y['running_avg_y'])))
    unfiltered_pos = np.array(list(zip(df_x['unfiltered_x'],
                                       df_y['unfiltered_y'])))

    # create CEP for each method
    cep_kalman, cep_running_avg, cep_unfiltered = calculate_cep(
        ideal_pos, kalman_pos, running_avg_pos, unfiltered_pos)

    # plot the cep as error circles of different colors in the x-y plane
    # Create a new figure
    fig, ax = plt.subplots()

    # Create circles with the given radii
    circle_kalman = plt.Circle((0, 0), cep_kalman, fill=False, label='Kalman',
                               color='r')
    circle_running_avg = plt.Circle((0, 0), cep_running_avg, fill=False,
                                    label='Running Avg', color='g')
    circle_unfiltered = plt.Circle((0, 0), cep_unfiltered, fill=False,
                                   label='Unfiltered', color='b')

    # Add the circles to the plot
    ax.add_artist(circle_kalman)
    ax.add_artist(circle_running_avg)
    ax.add_artist(circle_unfiltered)

    # Set the limits of the plot to show all circles
    ax.set_xlim(-max(cep_kalman, cep_running_avg, cep_unfiltered),
                max(cep_kalman, cep_running_avg, cep_unfiltered))
    ax.set_ylim(-max(cep_kalman, cep_running_avg, cep_unfiltered),
                max(cep_kalman, cep_running_avg, cep_unfiltered))

    # Add a legend
    plt.legend()

    # Add a grid
    plt.grid(True)

    # Set the y-axis label to 'Distance in Meters'
    plt.ylabel('Distance in Meters')

    # Set the x-axis label to 'Distance in Meters'
    plt.xlabel('Distance in Meters')


def plot_csv(file_path, type):
    # Read the CSV file into a DataFrame
    df = pd.read_csv(file_path)

    # Set the first column (time) as the index of the DataFrame
    df.set_index(df.columns[0], inplace=True)

    # Select the last three columns along with the index
    df_to_plot = df[df.columns[-3:-1]]

    # Rename the columns
    if type == 'x':
        df_to_plot.columns = ['kalman_x_error', 'current_x_error']
    elif type == 'y':
        df_to_plot.columns = ['kalman_y_error', 'current_y_error']

    # Plot the DataFrame
    df_to_plot.plot()

    # Add a grid
    plt.grid(True)

    # Set the y-axis label to 'Meters'
    plt.ylabel('Distance to Ideal in meters')

    # Set the x-axis label to 'Time'
    plt.xlabel('Time in seconds')

    # Separate the positive and negative vals
    positive_kalman_error = df_to_plot[df_to_plot.iloc[:, 0] > 0].iloc[:, 0]
    negative_kalman_error = df_to_plot[df_to_plot.iloc[:, 0] < 0].iloc[:, 0]

    positive_current_error = df_to_plot[df_to_plot.iloc[:, 1] > 0].iloc[:, 1]
    negative_current_error = df_to_plot[df_to_plot.iloc[:, 1] < 0].iloc[:, 1]

    # Calculate the average overshoot into the positive
    avg_positive_kalman_error = positive_kalman_error.mean()
    avg_positive_current_error = positive_current_error.mean()

    # Calculate the average overshoot into the negative
    avg_negative_kalman_error = negative_kalman_error.mean()
    avg_negative_current_error = negative_current_error.mean()

    # Add a horizontal line at the average val
    plt.axhline(y=avg_positive_kalman_error, color='b', linestyle='--')
    plt.axhline(y=avg_negative_kalman_error, color='b', linestyle='--')
    plt.axhline(y=avg_positive_current_error, color='r', linestyle='--')
    plt.axhline(y=avg_negative_current_error, color='r', linestyle='--')
    # plt.axhline(y=avg_kalman_error, color='r', linestyle='--')
    # plt.axhline(y=avg_current_error, color='b', linestyle='--')


def plot_csv_heading(file_path):
    # Read the CSV file into a DataFrame
    df = pd.read_csv(file_path)

    # Set the first column (time) as the index of the DataFrame
    df.set_index(df.columns[0], inplace=True)

    # Select the last two columns along with the index
    df_to_plot = df[df.columns[-2:-1]]

    # Rename the columns
    df_to_plot.columns = ['kalman_heading']

    # Plot the DataFrame
    df_to_plot.plot()

    # Add a grid
    plt.grid(True)

    # Set the y-axis label to 'Radians'
    plt.ylabel('Heading in Radians')

    # Set the x-axis label to 'Time'
    plt.xlabel('Time in seconds')

    # Separate the positive and negative vals
    positive_kalman_heading = df_to_plot[df_to_plot.iloc[:, 0] > 0].iloc[:, 0]
    negative_kalman_heading = df_to_plot[df_to_plot.iloc[:, 0] < 0].iloc[:, 0]
    # positive_current_heading =
    # df_to_plot[df_to_plot.iloc[:, 1] > 0].iloc[:, 1]
    # negative_current_heading =
    # df_to_plot[df_to_plot.iloc[:, 1] < 0].iloc[:, 1]

    # Calculate the average overshoot into the positive
    avg_positive_k_heading = positive_kalman_heading.mean()
    # avg_positive_c_heading = positive_current_heading.mean()

    # Calculate the average overshoot into the negative
    avg_negative_k_heading = negative_kalman_heading.mean()
    # avg_negative_c_heading = negative_current_heading.mean()

    # Add a horizontal line at the average val
    plt.axhline(y=avg_positive_k_heading, color='b', linestyle='--')
    plt.axhline(y=avg_negative_k_heading, color='b', linestyle='--')
    # plt.axhline(y=avg_positive_c_heading, color='r', linestyle='--')
    # plt.axhline(y=avg_negative_c_heading, color='r', linestyle='--')


def plot_csv_heading2(file_path):
    # Read the CSV file into a DataFrame
    df = pd.read_csv(file_path)

    # Set the first column (time) as the index of the DataFrame
    df.set_index(df.columns[0], inplace=True)

    # Select the first three columns
    df_to_plot = df[df.columns[:3]]

    # Rename the columns
    df_to_plot.columns = ['ideal_heading', 'kalman_heading', 'current_heading']

    # Plot the 'kalman_heading' and 'current_heading' columns with default
    # line style
    plt.plot(df_to_plot['kalman_heading'], label='kalman_heading')
    plt.plot(df_to_plot['current_heading'], label='current_heading')

    # Plot the 'ideal_heading' column with a blue dotted line
    plt.plot(df_to_plot['ideal_heading'], 'r:', label='ideal_heading')
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


def calculate_cep(ideal, kalman, running_avg, unfiltered, percentile=90):
    """
    Calculates the Circular Error Probable (CEP) for position data.

    Parameters:
    ideal (numpy.ndarray): The ideal positions.
    kalman (numpy.ndarray): The positions estimated using Kalman filtering.
    running_avg (numpy.ndarray): The positions calculated using a running
        average.
    unfiltered (numpy.ndarray): The unfiltered positions.
    percentile (int): The percentile to use when calculating the CEP.

    Returns:
    tuple: A tuple containing the CEP for each method.
    """
    # Calculate the errors
    error_kalman = np.sqrt(np.sum((kalman - ideal)**2, axis=1))
    error_running_avg = np.sqrt(np.sum((running_avg - ideal)**2, axis=1))
    error_unfiltered = np.sqrt(np.sum((unfiltered - ideal)**2, axis=1))

    # Calculate the CEP for each method
    cep_kalman = np.percentile(error_kalman, percentile)
    cep_running_avg = np.percentile(error_running_avg, percentile)
    cep_unfiltered = np.percentile(error_unfiltered, percentile)

    return cep_kalman, cep_running_avg, cep_unfiltered


def calculate_RMSPE_x_or_y(ideal, estimated):
    """
    Calculates the Root Mean Squared Percentage Error (RMSPE)
    for x or y data.

    Parameters:
    ideal (numpy.ndarray): The ideal x or y positions.
    estimated (numpy.ndarray): The estimated x or y positions.

    Returns:
    Tuple: A tuple containing the RMSPE and the error for each position.
    """

    # Replace zeros in 'ideal' array with a small number to avoid
    # division by zero
    ideal_no_zero = np.where(ideal == 0, 1e-10, ideal)
    # Calculate the errors
    error = ((ideal_no_zero - estimated) / ideal_no_zero) ** 2

    # Calculate the RMSPE
    rmspe = np.sqrt(np.mean(error))

    return rmspe, error


# TODO macht nicht viel Sinn -> entfernen
def calculate_RMSPE(ideal, estimated):
    """
    Calculates the Root Mean Squared Percentage Error (RMSPE)
    for position data.

    Parameters:
    ideal (numpy.ndarray): The ideal positions.
    estimated (numpy.ndarray): The estimated positions.

    Returns:
    Tuple: A tuple containing the RMSPE and the error for each position.
    """
    # Calculate the errors
    errors = np.sqrt(np.sum((ideal - estimated)**2, axis=1))

    # Calculate the RMSPE
    rmspe = np.sqrt(np.mean(errors**2))

    return rmspe, errors
# endregion CALCUATIONS


# plot_csv(file_path_x, 'x')
# plot_csv(file_path_y, 'y')


# plot_csv_heading(file_path_heading)
# plot_MSE_0()

# plot_best_tuned_file()
# plot_csv_heading2(file_path_heading)
# Show the plot
# plot_best_tuned_file_by_type('x', 'MSE')
# plot_best_tuned_file_by_type('y', 'MSE')
# plot_best_tuned_file_by_type('x', 'MAE')
# plot_best_tuned_file_by_type('y', 'MAE')
# plot_best_tuned_file_by_type('x', 'RMSPE')
# plot_best_tuned_file_by_type('y', 'RMSPE')

# plot_x_or_y_notched_box("data_26.csv", type='x', error_type='MSE')
# plot_x_or_y_notched_box("data_25.csv", type='x', error_type='MSE')
# plot_x_or_y_notched_box("data_25.csv", type='y', error_type='MSE')
# plot_x_or_y_notched_box("data_26.csv", type='y', error_type='MSE')
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
