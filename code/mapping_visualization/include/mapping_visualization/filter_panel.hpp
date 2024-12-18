#pragma once

#include <ros/ros.h>
#include <rviz/panel.h>

/**
 *  Include header generated from ui file
 *  Note that you will need to use add_library function first
 *  in order to generate the header file from ui.
 */
#include <ui_filter_panel.h>

namespace filter_panel
{

    /**
     *  Here we declare our new subclass of rviz::Panel. Every panel which
     *  can be added via the Panels/Add_New_Panel menu is a subclass of
     *  rviz::Panel.
     */

    class filterPanel : public rviz::Panel
    {
        /**
         * This class uses Qt slots and is a subclass of QObject, so it needs
         * the Q_OBJECT macro.
         */
        Q_OBJECT

    public:
        /**
         *  QWidget subclass constructors usually take a parent widget
         *  parameter (which usually defaults to 0).  At the same time,
         *  pluginlib::ClassLoader creates instances by calling the default
         *  constructor (with no arguments). Taking the parameter and giving
         *  a default of 0 lets the default constructor work and also lets
         *  someone using the class for something else to pass in a parent
         *  widget as they normally would with Qt.
         */

        filterPanel(QWidget *parent = 0);
        void onInitialize() override;
        void onUpdate();

        /**
         *  Now we declare overrides of rviz::Panel functions for saving and
         *  loading data from the config file.  Here the data is the topic name.
         */
        virtual void save(rviz::Config config) const;
        virtual void load(const rviz::Config &config);

        /**
         *  Next come a couple of public Qt Slots.
         */
    public Q_SLOTS:

        /**
         *  Here we declare some internal slots.
         */
    private Q_SLOTS:
        void is_collider_box(int state);
        void is_tracked_box(int state);
        void is_stopmark_box(int state);
        void is_lanemark_box(int state);
        void is_ignored_box(int state);

        /**
         *  Finally, we close up with protected member variables
         */
    protected:
        // UI pointer
        std::shared_ptr<Ui::filter_panel> ui_;

        ros::ServiceClient is_collider_client;
        ros::ServiceClient is_tracked_client;
        ros::ServiceClient is_stopmark_client;
        ros::ServiceClient is_lanemark_client;
        ros::ServiceClient is_ignored_client;
        ros::NodeHandle nh_;
    };
} // namespace