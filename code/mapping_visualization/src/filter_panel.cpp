#include "mapping_visualization/filter_panel.hpp"
#include <pluginlib/class_list_macros.hpp>

#include "rviz/visualization_manager.h"
#include "rviz/display_group.h"

#include "std_srvs/SetBool.h"
#include <QPalette>
#include <boost/bind.hpp>
PLUGINLIB_EXPORT_CLASS(filter_panel::filterPanel, rviz::Panel)

namespace filter_panel
{

    filterPanel::filterPanel(QWidget *parent)
        : rviz::Panel(parent),
          ui_(std::make_shared<Ui::filter_panel>())
    {
        // Extend the widget with all attributes and children from UI file
        ui_->setupUi(this);

        this->is_collider_client = this->nh_.serviceClient<std_srvs::SetBool>("vis/set_is_collider");
        this->is_tracked_client = this->nh_.serviceClient<std_srvs::SetBool>("vis/set_is_tracked");
        this->is_stopmark_client = this->nh_.serviceClient<std_srvs::SetBool>("vis/set_is_stopmark");
        this->is_lanemark_client = this->nh_.serviceClient<std_srvs::SetBool>("vis/set_is_lanemark");
        this->is_ignored_client = this->nh_.serviceClient<std_srvs::SetBool>("vis/set_is_ignored");

        QObject::connect(ui_->checkbox_collider, &QCheckBox::stateChanged, this, &filterPanel::is_collider_box);
        QObject::connect(ui_->checkbox_tracked, &QCheckBox::stateChanged, this, &filterPanel::is_tracked_box);
        QObject::connect(ui_->checkbox_stopmark, &QCheckBox::stateChanged, this, &filterPanel::is_stopmark_box);
        QObject::connect(ui_->checkbox_lanemark, &QCheckBox::stateChanged, this, &filterPanel::is_lanemark_box);
        QObject::connect(ui_->checkbox_ignored, &QCheckBox::stateChanged, this, &filterPanel::is_ignored_box);
    }

    void filterPanel::onInitialize()
    {
        // Through this we can continuously update the size
        connect(vis_manager_, &rviz::VisualizationManager::preUpdate, this, &filterPanel::onUpdate);
    }

    void filterPanel::onUpdate()
    {
        ui_->layoutWidget->resize(this->width(), this->height());
    }

    void filterPanel::is_collider_box(int state)
    {
        bool setting = ui_->checkbox_collider->isChecked();
        std_srvs::SetBool srv;
        srv.request.data = setting;
        this->is_collider_client.call(srv);
    }

    void filterPanel::is_tracked_box(int state)
    {
        bool setting = ui_->checkbox_tracked->isChecked();
        std_srvs::SetBool srv;
        srv.request.data = setting;
        this->is_tracked_client.call(srv);
    }
    void filterPanel::is_stopmark_box(int state)
    {
        bool setting = ui_->checkbox_stopmark->isChecked();
        std_srvs::SetBool srv;
        srv.request.data = setting;
        this->is_stopmark_client.call(srv);
    }
    void filterPanel::is_lanemark_box(int state)
    {
        bool setting = ui_->checkbox_lanemark->isChecked();
        std_srvs::SetBool srv;
        srv.request.data = setting;
        this->is_lanemark_client.call(srv);
    }
    void filterPanel::is_ignored_box(int state)
    {
        bool setting = ui_->checkbox_ignored->isChecked();
        std_srvs::SetBool srv;
        srv.request.data = setting;
        this->is_ignored_client.call(srv);
    }

    /**
     *  Save all configuration data from this panel to the given
     *  Config object. It is important here that you call save()
     *  on the parent class so the class id and panel name get saved.
     */
    void filterPanel::save(rviz::Config config) const
    {
        rviz::Panel::save(config);
    }

    /**
     *  Load all configuration data for this panel from the given Config object.
     */
    void filterPanel::load(const rviz::Config &config)
    {
        rviz::Panel::load(config);
    }
} // namespace filter_panel