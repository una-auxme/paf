import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

"""
The documentation on how to use this file can be found in
docs/perception/perception_heading_filter_debug_node.md
since it is used to visualize the data of the heading filter debug node.
"""

# FILE START and END are only needed for the range
# of files you want to search for the best tuned file
FILE_START = 0
FILE_END = 20

FILE_NUM = "00"  # Change this to plot your wanted file #


# current file_name to plot the data of one file!
file_name = "data_" + str(FILE_NUM) + ".csv"

# folder paths/ file paths
folder_path_x = "./x_error/"
folder_path_y = "./y_error/"
folder_path_heading = "./heading_error/"


# region PLOTS


def plot_best_tuned_file_by_type(type="x", error_type="MSE", check_type="IQR"):
    """
    Calculates the best tuned file by type and error type using a specific
    check type.

    Parameters:
    type (str): The type of data to read. Either 'x', 'y' or 'h'. (default 'x')
    error_type (str): The type of error to calculate. Either 'MSE', 'MAE' or
                    'RMSPE'. (default 'MSE')
    check_type (str): The type of error to calculate. Either 'IQR' or 'default'
                    IQR: Lower IQR is better
                    default: lowest MSE/MAE is better
                    Both are helpful to find the best tuned config.

    Returns:

    """
    best_file = ""
    best_val = []
    for i in range(FILE_START, FILE_END):
        if i < 10:
            file_name = "data_0" + str(i) + ".csv"
        else:
            file_name = "data_" + str(i) + ".csv"

        ideal, test_filter, current, unfiltered = get_x_or_y_or_h_from_csv_file(
            file_name, type
        )

        # calculate the error for each method by error_type
        if error_type == "MSE":
            # Calculate the MSE for each method
            val_test_filter, test_filter_list = calculate_mse_x_or_y_or_h(
                ideal, test_filter
            )
            val_current, current_list = calculate_mse_x_or_y_or_h(ideal, current)
            val_unfiltered, unfiltered_list = calculate_mse_x_or_y_or_h(
                ideal, unfiltered
            )
        elif error_type == "MAE":
            # Calculate the MAE for each method
            val_test_filter, test_filter_list = calculate_mae_x_or_y_or_h(
                ideal, test_filter
            )
            val_current, current_list = calculate_mae_x_or_y_or_h(ideal, current)
            val_unfiltered, unfiltered_list = calculate_mae_x_or_y_or_h(
                ideal, unfiltered
            )

        vals = [val_test_filter, val_current, val_unfiltered]

        # evaluate the best tuned file by check_type
        if check_type == "IQR":
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
        elif check_type == "default":
            if i == FILE_START:
                best_val = vals[0]
                best_file = file_name
            else:
                # compare the vals of the test_filter filter that
                # is tuned the best
                if abs(vals[0]) < abs(best_val):
                    best_val = vals[0]
                    best_file = file_name
                    # best_error_data = error_lists

    # plot the best file
    plot_x_or_y_or_h_notched_box(best_file, type=type, error_type=error_type)
    print(best_file)
    print(best_val)
    return


def plot_x_or_y_or_h_notched_box(file_name, type="x", error_type="MSE"):
    """
    Calculates and plots the error of x, y or heading data for any given
    error type.

    Parameters:
    file_name (str): The name of the CSV file.
    type (str): The type of data to read. Either 'x','y' or 'h'. (default 'x')
    error_type (str): The type of error to calculate.
                    'MSE', 'MAE' (default 'MSE')

    Returns:
    """
    if type == "x":
        ideal, test_filter, current, unfiltered = get_x_or_y_or_h_from_csv_file(
            file_name, "x"
        )
    elif type == "y":
        ideal, test_filter, current, unfiltered = get_x_or_y_or_h_from_csv_file(
            file_name, "y"
        )
    elif type == "h":
        ideal, test_filter, current, unfiltered = get_x_or_y_or_h_from_csv_file(
            file_name, "h"
        )

    if error_type == "MSE":
        # Calculate the MSE for each method
        val_test_filter, test_filter_list = calculate_mse_x_or_y_or_h(
            ideal, test_filter
        )
        val_current, current_list = calculate_mse_x_or_y_or_h(ideal, current)
        val_unfiltered, unfiltered_list = calculate_mse_x_or_y_or_h(ideal, unfiltered)
    elif error_type == "MAE":
        # Calculate the MAE for each method
        val_test_filter, test_filter_list = calculate_mae_x_or_y_or_h(
            ideal, test_filter
        )
        val_current, current_list = calculate_mae_x_or_y_or_h(ideal, current)
        val_unfiltered, unfiltered_list = calculate_mae_x_or_y_or_h(ideal, unfiltered)

    # Create a new figure
    fig, ax = plt.subplots()

    # Create a list of all errors
    error_list = [test_filter_list, current_list, unfiltered_list]
    # Create a box plot with notches
    boxplot = ax.boxplot(
        error_list,
        notch=True,
        labels=["Test Filter", "Current", "Unfiltered"],
        patch_artist=True,
    )

    # fill with colors and put median vals in the boxes
    colors = ["pink", "lightblue", "lightgreen"]

    tuple = zip(boxplot["boxes"], colors, boxplot["medians"], boxplot["whiskers"][::2])

    for i, (box, color, median, whiskers) in enumerate(tuple):
        box.set_facecolor(color)
        median_val = median.get_ydata()[1]
        ax.text(
            i + 1,
            median_val,
            f"Median: {median_val:.2f}",
            va="center",
            ha="center",
            backgroundcolor="white",
        )

        # Calculate IQR
        q3, q1 = np.percentile(error_list[i], [75, 0])
        iqr = q3 - q1

        # Get the y position for the IQR text
        median_y = boxplot["medians"][i].get_ydata()[0]  # height of the notch

        # Add the IQR text
        ax.text(
            i + 0.8,
            median_y,
            f"IQR: {iqr:.2f}",
            va="center",
            ha="center",
            rotation=90,
            color="red",
            backgroundcolor="white",
        )

    # Set the labels
    ax.set_xlabel("Filter")
    ax.set_ylabel(error_type)
    ax.set_title(error_type + " of " + type + " for different methods")
    ax.yaxis.grid(True)

    # Show the plot
    plt.show()


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
        get_positions_from_csv_file(file_name)
    )

    # Calculate the MSE for each method
    val_test_filter, test_filter_list = calculate_mse_pos(ideal_pos, test_filter_pos)
    val_current, current_list = calculate_mse_pos(ideal_pos, current_pos)
    val_unfiltered, unfiltered_list = calculate_mse_pos(ideal_pos, unfiltered_pos)

    # Create a new figure
    fig, ax = plt.subplots()

    # Create a list of all positions
    pos_mse_list = [test_filter_list, current_list, unfiltered_list]
    # Create a box plot with notches
    boxplot = ax.boxplot(
        pos_mse_list,
        notch=True,
        labels=["Test Filter", "Current", "Unfiltered"],
        patch_artist=True,
    )

    # fill with colors and put median vals in the boxes
    colors = ["pink", "lightblue", "lightgreen"]
    for i, (box, color, median) in enumerate(
        zip(boxplot["boxes"], colors, boxplot["medians"])
    ):
        box.set_facecolor(color)
        median_val = median.get_ydata()[1]
        ax.text(
            i + 1,
            median_val,
            f"Median: {median_val:.2f}",
            va="center",
            ha="center",
            backgroundcolor="white",
        )

    # Set the labels
    ax.set_xlabel("Method")
    ax.set_ylabel("MSE")
    ax.set_title("MSE for different methods")
    ax.yaxis.grid(True)

    # Show the plot
    plt.show()


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
        get_positions_from_csv_file(file_name)
    )

    # Calculate the MAE for each method
    mae_test_filter, test_filter_list = calculate_mae_pos(ideal_pos, test_filter_pos)
    mae_current, current_list = calculate_mae_pos(ideal_pos, current_pos)
    mae_unfiltered, unfiltered_list = calculate_mae_pos(ideal_pos, unfiltered_pos)

    # Create a new figure
    fig, ax = plt.subplots()

    # Create a list of all positions
    pos_mae_list = [test_filter_list, current_list, unfiltered_list]

    # Create a box plot with notches
    boxplot = ax.boxplot(
        pos_mae_list,
        notch=True,
        labels=["Test Filter", "Current", "Unfiltered"],
        patch_artist=True,
    )

    # fill with colors and put median vals in the boxes
    colors = ["pink", "lightblue", "lightgreen"]
    for i, (box, color, median) in enumerate(
        zip(boxplot["boxes"], colors, boxplot["medians"])
    ):
        box.set_facecolor(color)
        median_val = median.get_ydata()[1]
        ax.text(
            i + 1,
            median_val,
            f"Median: {median_val:.2f}",
            va="center",
            ha="center",
            backgroundcolor="white",
        )

    # Set the labels
    ax.set_xlabel("Method")
    ax.set_ylabel("MAE")
    ax.set_title("MAE for different methods")
    ax.yaxis.grid(True)

    # Show the plot
    plt.show()


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
    ideal_pos = np.array(list(zip(df_x["Ideal (Carla)"], df_y["Ideal (Carla)"])))
    test_filter_pos = np.array(list(zip(df_x["Test Filter"], df_y["Test Filter"])))
    current_pos = np.array(list(zip(df_x["Current"], df_y["Current"])))
    unfiltered_pos = np.array(list(zip(df_x["Unfiltered"], df_y["Unfiltered"])))

    # create CEP for each method
    cep_test_filter, cep_current, cep_unfiltered = calculate_cep(
        ideal_pos, test_filter_pos, current_pos, unfiltered_pos
    )

    # plot the cep as error circles of different colors in the x-y plane
    # Create a new figure
    fig, ax = plt.subplots()

    # Create circles with the given radii
    circle_test_filter = plt.Circle(
        (0, 0), cep_test_filter, fill=False, label="Test Filter", color="r"
    )
    circle_current = plt.Circle(
        (0, 0), cep_current, fill=False, label="Current", color="g"
    )
    circle_unfiltered = plt.Circle(
        (0, 0), cep_unfiltered, fill=False, label="Unfiltered", color="b"
    )

    # Add the circles to the plot
    ax.add_artist(circle_test_filter)
    ax.add_artist(circle_current)
    ax.add_artist(circle_unfiltered)

    # Set the limits of the plot to show all circles
    ax.set_xlim(
        -max(cep_test_filter, cep_current, cep_unfiltered),
        max(cep_test_filter, cep_current, cep_unfiltered),
    )
    ax.set_ylim(
        -max(cep_test_filter, cep_current, cep_unfiltered),
        max(cep_test_filter, cep_current, cep_unfiltered),
    )

    # Add a legend
    plt.legend()

    # Add a grid
    plt.grid(True)

    # Set the y-axis label to 'Distance in Meters'
    plt.ylabel("Distance in Meters")

    # Set the x-axis label to 'Distance in Meters'
    plt.xlabel("Distance in Meters")


def plot_csv_x_or_y(file_name, type="x"):
    """
    Plots the x or y data from a CSV file.

    Parameters:
    file_name (str): The name of the CSV file.
    """
    if type == "x":
        file_path = folder_path_x + file_name
    elif type == "y":
        file_path = folder_path_y + file_name

    # Read the CSV file into a DataFrame
    df = pd.read_csv(file_path)

    # Plot the 'test_filter' (blue) and 'current' (green)
    plt.plot(df["Test Filter"], "b-", label="Test Filter")
    plt.plot(df["Current"], "g-", label="Current")

    # Plot the 'ideal' column with a red dotted line
    plt.plot(df["Ideal (Carla)"], "r:", label="Ideal")

    # Display the legend
    plt.legend()
    # Plot the DataFrame
    df.plot()

    # Add a grid
    plt.grid(True)

    # Set the y-
    # axis label to 'Distance in Meters'
    plt.ylabel("Distance in Meters")

    # Set the x-axis label to 'Time'
    plt.xlabel("Time in seconds")

    plt.title(type + " Positions in Meters")


def plot_csv_heading(file_name):
    """
    Plots the heading data from a CSV file.

    Parameters:
    file_name (str): The name of the CSV file.

    Returns:
    """
    file_path_heading = folder_path_heading + file_name
    # Read the CSV file into a DataFrame
    df = pd.read_csv(file_path_heading)

    # Plot the 'test_filter_heading' (blue) and 'current_heading' (green)
    # line style
    plt.plot(df["Test Filter"], "b-", label="Test Filter Heading")
    plt.plot(df["Current"], "g-", label="Current Heading")

    # Plot the 'ideal_heading' column with a blue dotted line
    plt.plot(df["Ideal (Carla)"], "r:", label="Ideal Heading")
    # Display the legend
    plt.legend()
    # Plot the DataFrame
    df.plot()

    # Add a grid
    plt.grid(True)

    # Set the y-axis label to 'Radians'
    plt.ylabel("Heading in Radians")

    # Set the x-axis label to 'Time'
    plt.xlabel("Time in seconds")


def plot_csv_positions(file_name):
    """
    Plots the x AND y data from a CSV file.

    Parameters:
    file_name (str): The name of the CSV file.

    Returns:
    """
    # Read the CSV file into a DataFrame
    ideal, test_filter, current, unfiltered = get_positions_from_csv_file(file_name)

    ideal_x, ideal_y = zip(*ideal)
    test_filter_x, test_filter_y = zip(test_filter)
    current_x, current_y = zip(current)
    unfiltered_x, unfiltered_y = zip(unfiltered)

    plt.plot(ideal_x, ideal_y, marker=",", color="red", label="Ideal")
    plt.plot(
        test_filter_x, test_filter_y, marker=".", color="blue", label="Test Filter"
    )
    plt.plot(current_x, current_y, marker=".", color="green", label="Current")
    plt.plot(unfiltered_x, unfiltered_y, marker=".", color="purple", label="Unfiltered")

    # Display the legend
    plt.legend()

    # Add a grid
    plt.grid(True)

    # Set the y-axis label to 'Y Position in Meters'
    plt.ylabel("Y Position in Meters")

    # Set the x-axis label to 'X Position in Meters'
    plt.xlabel("X Position in Meters")

    plt.title("X and Y Positions in Meters")


# endregion PLOTS


# region CALCUATIONS
def calculate_mae_pos(ideal, estimated):
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


def calculate_mse_pos(ideal, estimated):
    """
    Calculates the Mean Squared Error (MSE) for position data.

    Parameters:
    ideal (numpy.ndarray): The ideal positions.
    estimated (numpy.ndarray): The estimated positions.

    Returns:
    Tuple: A tuple containing the MSE and the error for each position.
    """
    # Calculate the errors
    error = np.linalg.norm(ideal - estimated, axis=1) ** 2

    # Calculate the MSE
    mse = np.mean(error)

    return mse, error


def calculate_mae_x_or_y_or_h(ideal, estimated):
    """
    Calculates the Mean Absolute Error (MAE) for x or y or heading data.

    Parameters:
    ideal (numpy.ndarray): The ideal x or y positions or heading.
    estimated (numpy.ndarray): The estimated x or y positions or heading.

    Returns:
    Tuple: A tuple containing the MAE and the error for each position
        or heading.
    """
    # Calculate the errors
    error = np.abs(ideal - estimated)

    # Calculate the MAE
    mae = np.mean(error)

    return mae, error


def calculate_mse_x_or_y_or_h(ideal, estimated):
    """
    Calculates the Mean Squared Error (MSE) for x or y or heading data.

    Parameters:
    ideal (numpy.ndarray): The ideal x or y positions oe heading.
    estimated (numpy.ndarray): The estimated x or y positions or heading.

    Returns:
    Tuple: A tuple containing the MSE and the error for each position
        or heading.
    """
    # Calculate the errors
    error = (ideal - estimated) ** 2

    # Calculate the MSE
    mse = np.mean(error)

    return mse, error


def calculate_cep(ideal, test_filter, current, unfiltered, percentile=90):
    """
    Calculates the Circular Error Probable (CEP) for position data.
    Comparable to the Error Circle that Google Maps shows for position
    uncertainty.

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
    error_test_filter = np.sqrt(np.sum((test_filter - ideal) ** 2, axis=1))
    error_current = np.sqrt(np.sum((current - ideal) ** 2, axis=1))
    error_unfiltered = np.sqrt(np.sum((unfiltered - ideal) ** 2, axis=1))

    # Calculate the CEP for each method
    cep_test_filter = np.percentile(error_test_filter, percentile)
    cep_current = np.percentile(error_current, percentile)
    cep_unfiltered = np.percentile(error_unfiltered, percentile)

    return cep_test_filter, cep_current, cep_unfiltered


# endregion CALCUATIONS


# region helper methods:
def get_positions_from_csv_file(file_name, file_name_y=file_name):
    """
    Reads position data from CSV files and returns them as numpy arrays.

    Args:
        file_name (str): The name of the CSV file
        file_name_y (str): The name of the CSV file for the y data.
                           (default file_name; should be the same)

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
    file_path_y = folder_path_y + file_name_y

    # Read the CSV file into a DataFrame
    df_x = pd.read_csv(file_path_x)
    df_y = pd.read_csv(file_path_y)

    # Set the first column (time) as the index of the DataFrames
    df_x.set_index(df_x.columns[0], inplace=True)
    df_y.set_index(df_y.columns[0], inplace=True)

    # create pos tuples of the x and y data and store them as numpy arrays
    ideal_pos = np.array(list(zip(df_x["Ideal (Carla)"], df_y["Ideal (Carla)"])))
    test_filter_pos = np.array(list(zip(df_x["Test Filter"], df_y["Test Filter"])))
    current_pos = np.array(list(zip(df_x["Current"], df_y["Current"])))
    unfiltered_pos = np.array(list(zip(df_x["Unfiltered"], df_y["Unfiltered"])))

    return ideal_pos, test_filter_pos, current_pos, unfiltered_pos


def get_x_or_y_or_h_from_csv_file(file_name, type="x"):
    """
    Reads x,y or heading data from CSV files and returns them as numpy arrays.

    Args:
        file_name (str): The name of the CSV file.
        type (str): The type of data to read. Either 'x','y' or 'h'.
                    (default 'x')

    Returns:
        tuple: A tuple containing four numpy arrays representing the positions.
            - ideal: The ideal data.
            - test_filter: The data estimated using test_filter filtering.
            - current: The data calculated using a running
            average.
            - unfiltered: The unfiltered data."""

    if type == "x":
        file_path = folder_path_x + file_name
    elif type == "y":
        file_path = folder_path_y + file_name
    elif type == "h":
        file_path = folder_path_heading + file_name

    # Read the CSV file into a DataFrame
    # (skip the first couple rows because of wrong measurements)
    df = pd.read_csv(file_path)

    # Set the first column (time) as the index of the DataFrames
    df.set_index(df.columns[0], inplace=True)

    if type == "x":
        # store x as numpy arrays
        ideal = np.array(df["Ideal (Carla)"])
        test_filter = np.array(df["Test Filter"])
        current = np.array(df["Current"])
        unfiltered = np.array(df["Unfiltered"])
    elif type == "y":
        # store y as numpy arrays
        ideal = np.array(df["Ideal (Carla)"])
        test_filter = np.array(df["Test Filter"])
        current = np.array(df["Current"])
        unfiltered = np.array(df["Unfiltered"])
    elif type == "h":
        # store heading as numpy arrays
        ideal = np.array(df["Ideal (Carla)"])
        test_filter = np.array(df["Test Filter"])
        current = np.array(df["Current"])
        unfiltered = np.array(df["Unfiltered"])

    return ideal, test_filter, current, unfiltered


# endregion helper methods


# main:
if __name__ == "__main__":
    # file_name can be changed by changing the FILE_NUM variable
    data = file_name
    plot_CEP(data)

    plot_x_or_y_or_h_notched_box(data, type="x", error_type="MSE")
    plot_x_or_y_or_h_notched_box(data, type="x", error_type="MAE")

    plot_x_or_y_or_h_notched_box(data, type="y", error_type="MSE")
    plot_x_or_y_or_h_notched_box(data, type="y", error_type="MAE")

    # plot_x_or_y_or_h_notched_box(data, type='h', error_type='MSE')
    # plot_x_or_y_or_h_notched_box(data, type='h', error_type='MAE')

    # plot_best_tuned_file_by_type(type='x',error_type='MSE',check_type='IQR')
    # plot_best_tuned_file_by_type(type='y',error_type='MSE',check_type='IQR')
    # plot_best_tuned_file_by_type(type='h',error_type='MSE',check_type='IQR')

    # plot_csv_x_or_y(data, type='x')
    # plot_csv_x_or_y(data, type='y')
    # plot_csv_heading(data)
    # plot_csv_positions(data)

    # always use plt.show() to show the plots
    plt.show()
