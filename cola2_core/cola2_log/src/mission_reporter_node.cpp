#include <cola2_lib_ros/param_loader.h>
#include <cola2_lib_ros/this_node.h>
#include <cola2_msgs/CaptainStatus.h>
#include <cola2_msgs/RecoveryAction.h>
#include <cola2_msgs/SafetySupervisorStatus.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <sys/statvfs.h>

#include <ctime>
#include <experimental/filesystem>
#include <fstream>
#include <iomanip>
#include <regex>

// Captain notificaion of mission endings
static const std::string mission_finalized_string = "Mission finalized: ";       //!< Mission finalized string.
static const std::string mission_paused_string = "Mission paused: ";             //!< Mission paused string.
static const std::string mission_resume_string = "Resuming mission: ";           //!< Resuming mission string.
static const std::string mission_disabled_string = "Active mission disabled: ";  //!< Active mission disabled.
static const std::string external_mission_disabled_string =
    "External mission disabled: ";  //!< External mission disabled string.

/**
 * @brief Get the current mission name.
 *
 * @param msg Captain status
 * @return std::string Current mission name if the captain is in mission or external mission. Empty string otherwise.
 */
std::string getCurrentMissionFromMessage(const cola2_msgs::CaptainStatusConstPtr msg)
{
  // Check if captain is in mission.
  if (msg->state == cola2_msgs::CaptainStatus::MISSION)
  {
    std::vector<cola2_msgs::MissionState> missions = msg->loaded_missions;
    std::sort(missions.begin(), missions.end(),
              [](const cola2_msgs::MissionState& lhs, const cola2_msgs::MissionState& rhs) -> bool {
                return lhs.last_active > rhs.last_active;
              });
    return missions.front().name;
  }
  if (msg->state == cola2_msgs::CaptainStatus::EXTERNALMISSION)
  {
    std::string key_string("External mission caller name: ");
    std::size_t found = msg->message.find(key_string);
    if (found != std::string::npos)
    {
      return msg->message.substr(found + key_string.length());
    }
  }
  return "";
}

/**
 * @brief Missing ending reasons.
 */
enum class MissionEndingReasons
{
  Ongoing,             //!< Ongoing mission.
  Successful,          //!< Mission finalized successfully.
  Paused,              //!< Mission paused.
  Disabled,            //!< Mission disabled.
  Safety,              //!< Mission aborted for safety reason.
  FinishedAfterSafety  //!< Mission aborted for safety reason and the safety situation has been resolved.
};

/**
 * @brief Operator to ease the printing of the MissionEndingReasons.
 *
 * @param os Output stream.
 * @param reason Mission ending reason.
 * @return Output stream
 */
std::ostream& operator<<(std::ostream& os, const MissionEndingReasons& reason)
{
  switch (reason)
  {
    case MissionEndingReasons::Ongoing:
      os << "Ongoing";
      break;
    case MissionEndingReasons::Successful:
      os << "Successful";
      break;
    case MissionEndingReasons::Paused:
      os << "Paused";
      break;
    case MissionEndingReasons::Disabled:
      os << "Disabled";
      break;
    case MissionEndingReasons::Safety:
      os << "Safety";
      break;
    case MissionEndingReasons::FinishedAfterSafety:
      os << "Finished after safety";
      break;
    default:
      os << "Unknown";
  }
  return os;
}

/**
 * @brief Structure with the mission reporting data.
 */
struct MissionReport
{
  typedef std::shared_ptr<MissionReport> Ptr;
  std::string mission_name;                        //!< Mission name.
  std::vector<rosgraph_msgs::LogConstPtr> rosout;  //!< Rosout data.
  std::vector<ros::Time> start_time;               //!< Start/resume mission time. Vector to account for pauses.
  std::vector<ros::Time> end_time;     //!< End/pause/abort/cancel mission time. Vector to account for pauses.
  MissionEndingReasons ending_reason;  //!< Mission ending reason.
  cola2_msgs::SafetySupervisorStatusPtr safety_msg =
      nullptr;  //<! Safety message that produced the mission ending if there was any.
};

/**
 * @brief Mission reporter node class.
 *
 */
class MissionReporterNode
{
private:
  // ROS publishers, subscrivers and services.
  ros::NodeHandle nh_;                        //!< Node handler.
  ros::Subscriber subs_rosout_;               //!< Rosout subscriber.
  ros::Subscriber subs_captain_status_;       //!< Captain status subscriber.
  ros::Subscriber subs_safety_status_;        //!< Safety supervisor status subscriber.
  ros::ServiceServer srv_get_last_report_;    //!< Get last report service server.
  ros::ServiceServer srv_reset_last_report_;  //!< Reset last report service server.

  // Captain state.
  std::string last_mission_ = "";  //!< Name of the last mission.
  bool safety_flag_ = false;

  // Reports
  std::vector<MissionReport::Ptr> mission_reports_;       //<! List of active reports.
  std::string report_folder_ = "";                        //!< Reporting folder.
  std::string report_file_name_ = "";                     //!< Report file name.
  std::vector<std::string> filter_nodes_;                 //!< Nodes to filter out.
  std::regex filter_regex_;                               //!< Regex to filter out rosout comments.
  bool rosout_include_nanoseconds_ = true;                //!< Include nanoseconds in rosout time format.
  std::string rosout_time_format_ = "%Y/%m/%d %H:%M:%S";  //!< Rosout time format.

  /**
   * @brief Get the Report with the given mission name.
   *
   * @param mission_name Name of the mission.
   * @return Mission report pointer.
   */
  MissionReport::Ptr getReport(const std::string& mission_name) const;

  /**
   * @brief Get the current report.
   *
   * @return Mission report pointer.
   */
  MissionReport::Ptr getCurrentReport() const;

  /**
   * @brief Updates the last time of the given report and the disk usage.
   *
   * @param report Mission report to update.
   * @param stamp Last report time.
   */
  void updateLastTimeAndDiskStatus(MissionReport::Ptr report, const ros::Time& stamp = ros::Time::now());

  /**
   * @brief Callback for rosout.
   *
   * @param msg Rosout message.
   */
  void cbkRosout(const rosgraph_msgs::LogConstPtr msg);

  /**
   * @brief Captain status callback.
   *
   * @param msg Captain status message.
   */
  void cbkCaptainStatus(const cola2_msgs::CaptainStatusConstPtr msg);

  /**
   * @brief Safety supervisor status callback.
   *
   * @param msg Safety supervisor status message.
   */
  void cbkSafetyStatus(const cola2_msgs::SafetySupervisorStatusPtr msg);

  /**
   * @brief Get last report service callback.
   *
   * @param req Request.
   * @param res Response
   * @return Success of operation.
   */
  bool cbkGetLastReportSrv(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);

  /**
   * @brief Reset last report service.
   *
   * @param req Request.
   * @param res Response
   * @return Success of operation.
   */
  bool cbkResetLastReportSrv(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);

  /**
   * @brief Get the ending message for a given report.
   *
   * @param report Report to generate the ending message.
   * @return std::string Ending message for the report.
   */
  std::string getEndingMessage(const MissionReport::Ptr report) const;

  /**
   * @brief Generate a report markdown as string.
   *
   * @return std::string
   */
  std::string generateReport();

  /**
   * @brief Save the current report.
   */
  void saveReport();

  /**
   * @brief Get the configuration from the parameters server.
   *
   * @return Operation success.
   */
  bool getConfig();

public:
  /**
   * @brief Constructor.
   *
   */
  MissionReporterNode();

  /**
   * @brief Spin the ros node.
   */
  void spin() const;
};

/**
 * @brief Convert a ROS stamp into a string using the provided format.
 *
 * @param stamp ROS stamp.
 * @param format format as in strftime.
 * @return std::string
 */
std::string stampToString(const ros::Time& stamp, const std::string format)
{
  const int output_size = 100;
  char output[output_size];
  std::time_t raw_time = static_cast<time_t>(stamp.sec);
  struct tm* timeinfo = localtime(&raw_time);
  std::strftime(output, output_size, format.c_str(), timeinfo);
  return std::string(output);
}

MissionReport::Ptr MissionReporterNode::getReport(const std::string& mission_name) const
{
  for (auto report : mission_reports_)
  {
    if (report->mission_name == mission_name)
    {
      return report;
    }
  }
  return nullptr;
}

MissionReport::Ptr MissionReporterNode::getCurrentReport() const
{
  MissionReport::Ptr report;
  if (last_mission_ != "")
  {
    auto report = getReport(last_mission_);
    if (report && (report->ending_reason == MissionEndingReasons::Ongoing ||
                   report->ending_reason == MissionEndingReasons::Paused ||
                   report->ending_reason == MissionEndingReasons::Disabled ||
                   report->ending_reason == MissionEndingReasons::Safety))
    {
      return report;
    }
  }
  return nullptr;
}

MissionReporterNode::MissionReporterNode() : nh_("~")
{
  subs_rosout_ = nh_.subscribe("/rosout", 1000, &MissionReporterNode::cbkRosout, this);
  subs_captain_status_ = nh_.subscribe(cola2::ros::getNamespace() + "/captain/captain_status", 1,
                                       &MissionReporterNode::cbkCaptainStatus, this);
  subs_safety_status_ = nh_.subscribe(cola2::ros::getNamespace() + "/safety_supervisor/status", 1,
                                      &MissionReporterNode::cbkSafetyStatus, this);
  srv_get_last_report_ = nh_.advertiseService("get_last_report", &MissionReporterNode::cbkGetLastReportSrv, this);
  srv_reset_last_report_ = nh_.advertiseService("reset_last_report", &MissionReporterNode::cbkResetLastReportSrv, this);
  while (!getConfig())
  {
    ROS_WARN_THROTTLE(10, "Error loading parameters");
    ros::spinOnce();
    ros::Rate(1).sleep();
  }
}

void MissionReporterNode::updateLastTimeAndDiskStatus(MissionReport::Ptr report, const ros::Time& stamp)
{
  if (!report)
  {
    return;
  }
  report->end_time.back() = stamp;
}

void MissionReporterNode::cbkRosout(const rosgraph_msgs::LogConstPtr msg)
{
  // Get mission report
  MissionReport::Ptr report = getCurrentReport();
  if (!report)
  {
    return;
  }

  // Check captain messages for mission ending/pause...
  bool save_report = false;
  if (msg->name.find("captain") != std::string::npos)
  {
    const std::string message = msg->msg;
    size_t idx = message.find(mission_finalized_string);
    if (idx != std::string::npos && !safety_flag_)
    {
      if (message.substr(idx + mission_finalized_string.size()) == last_mission_)
      {
        report->ending_reason = MissionEndingReasons::Successful;
        save_report = true;
        ROS_INFO_STREAM("Mission " << report->mission_name << " finished correctly");
      }
    }
    idx = message.find(mission_paused_string);
    if (idx != std::string::npos && !safety_flag_)
    {
      if (message.substr(idx + mission_paused_string.size()) == last_mission_)
      {
        report->ending_reason = MissionEndingReasons::Paused;
        save_report = true;
        ROS_INFO_STREAM("Mission " << report->mission_name << " paused");
      }
    }
    idx = message.find(mission_disabled_string);
    if (idx != std::string::npos && !safety_flag_)
    {
      if (message.substr(idx + mission_disabled_string.size()) == last_mission_)
      {
        report->ending_reason = MissionEndingReasons::Disabled;
        save_report = true;
        ROS_INFO_STREAM("Mission " << report->mission_name << " disabled");
      }
    }
    idx = message.find(mission_resume_string);
    if (idx != std::string::npos && !safety_flag_)
    {
      if (message.substr(idx + mission_resume_string.size()) == last_mission_)
      {
        report->ending_reason = MissionEndingReasons::Ongoing;
        save_report = true;
        ROS_INFO_STREAM("Mission " << report->mission_name << " ongoing");
      }
    }
    idx = message.find(external_mission_disabled_string);
    if (idx != std::string::npos && !safety_flag_)
    {
      if (message.substr(idx + external_mission_disabled_string.size()) == last_mission_)
      {
        report->ending_reason = MissionEndingReasons::Successful;
        save_report = true;
        ROS_INFO_STREAM("Mission " << report->mission_name << " finished");
      }
    }
  }

  // Check if it is an exclusion node or if it is excluded by regex expression
  bool exclusion_node = std::find_if(filter_nodes_.begin(), filter_nodes_.end(), [msg](const std::string& str) {
                          return msg->name.find(str) != std::string::npos;
                        }) != filter_nodes_.end();
  bool exclusion_regex = std::regex_search(msg->msg, filter_regex_);
  if (exclusion_node || exclusion_regex)
  {
    if (save_report)
    {
      updateLastTimeAndDiskStatus(report, msg->header.stamp);
      saveReport();
    }
    return;
  }

  // Append to the log
  report->rosout.push_back(msg);
  saveReport();
}

void MissionReporterNode::cbkCaptainStatus(const cola2_msgs::CaptainStatusConstPtr msg)
{
  std::string current_mission = getCurrentMissionFromMessage(msg);
  if (current_mission != "" && last_mission_ != current_mission)
  {
    MissionReport::Ptr report = getReport(current_mission);
    bool new_report = !report;
    if (new_report)
    {
      // New report.
      report = std::make_shared<MissionReport>();
      report->mission_name = current_mission;
      ROS_INFO_STREAM("Creating new mission report: " << current_mission);
    }
    else
    {
      ROS_INFO_STREAM("Reopening mission report: " << report->mission_name);
    }
    report->ending_reason = MissionEndingReasons::Ongoing;
    // If no new report is necessary, the mission was paused and is now resumed.
    report->start_time.push_back(ros::Time::now());
    report->end_time.push_back(report->start_time.back());
    if (new_report)
    {
      // Only add the report if all the process was successful.
      mission_reports_.push_back(report);
    }
    last_mission_ = current_mission;
    ROS_INFO_STREAM("Setting last mission to " << current_mission);
    saveReport();
  }
  else if (last_mission_ == current_mission && current_mission != "" && !safety_flag_)
  {
    // Update mission.
    MissionReport::Ptr report = getCurrentReport();
    if (!report)
    {
      ROS_ERROR("Error getting mission report for update normal captain callback.");
      return;
    }
    updateLastTimeAndDiskStatus(report);
    saveReport();
  }
  else if (last_mission_ != "" && safety_flag_)
  {
    // Update mission.
    MissionReport::Ptr report = getCurrentReport();
    if (!report)
    {
      ROS_ERROR("Error getting mission report for update captain callback during safety event.");
      return;
    }
    updateLastTimeAndDiskStatus(report);
    saveReport();
  }
}

void MissionReporterNode::cbkSafetyStatus(const cola2_msgs::SafetySupervisorStatusPtr msg)
{
  if (!safety_flag_ && msg->recovery_action.error_level > cola2_msgs::RecoveryAction::INFORMATIVE)
  {
    safety_flag_ = true;  // Set it before get current report as it is used to select the last report if
                          // none matches the current mission.
    MissionReport::Ptr report = getCurrentReport();
    if (!report)
    {
      ROS_DEBUG("Did not get any mission report.");
      safety_flag_ = false;  // Undo the change as it is not yet properly initialized.
      return;
    }
    ROS_INFO_STREAM("Mission " << report->mission_name << " safety!");
    report->ending_reason = MissionEndingReasons::Safety;
    updateLastTimeAndDiskStatus(report, msg->header.stamp);
    report->safety_msg = msg;
  }
  else if (msg->recovery_action.error_level <= cola2_msgs::RecoveryAction::INFORMATIVE && safety_flag_)
  {
    MissionReport::Ptr report = getCurrentReport();
    if (report)
    {
      report->ending_reason = MissionEndingReasons::FinishedAfterSafety;
      ROS_INFO_STREAM("Mission " << report->mission_name << " safety situation finalized.");
    }
    safety_flag_ = false;
  }
}

bool MissionReporterNode::cbkGetLastReportSrv(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
  if (report_file_name_ != "")
  {
    res.message = report_folder_ + "/" + report_file_name_;
    res.success = true;
  }
  else
  {
    res.message = "";
    res.success = false;
  }
  return true;
}

bool MissionReporterNode::cbkResetLastReportSrv(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
  ROS_INFO("Reset report.");
  saveReport();
  mission_reports_.clear();
  report_file_name_ = "";
  last_mission_ = "";
  res.success = true;
  return true;
}

std::string durationToHumanReadableString(const int32_t& duration_sec)
{
  std::stringstream ss;
  int32_t sec = duration_sec;
  if (sec >= 60)
  {
    int32_t min = sec / 60;
    sec = sec % 60;
    if (min >= 60)
    {
      int32_t hour = min / 60;
      min = min % 60;
      if (hour >= 24)
      {
        int32_t days = hour / 24;
        hour = hour % 24;
        ss << days << "d ";
      }
      ss << hour << "h ";
    }
    ss << min << "m ";
  }
  ss << sec << "s";
  return ss.str();
}

std::string memoryToHumanReadableString(const int memory)
{
  int i = 0;
  std::vector<std::string> mem_suffix = { "b", "Kb", "Mb", "Gb", "Tb" };
  bool negative = memory < 0;
  int mem = memory;
  if (negative)
  {
    mem *= -1;
  }
  while (mem >= 1024 && i < mem_suffix.size() - 1)
  {
    mem /= 1024;
    i++;
  }
  if (negative)
  {
    mem *= -1;
  }
  return std::to_string(mem) + mem_suffix[i];
}

void MissionReporterNode::saveReport()
{
  if (mission_reports_.size() == 0)
  {
    return;
  }

  // Generate report name
  if (report_file_name_ == "")
  {
    report_file_name_ = stampToString(mission_reports_.front()->start_time.front(), "%Y-%m-%d_%H-%M-%S") + "_mission_"
                                                                                                           "report.md";
  }

  // Check if output folder exists
  if (!std::experimental::filesystem::exists(report_folder_))
  {
    if (!std::experimental::filesystem::create_directories(report_folder_))
    {
      ROS_ERROR_STREAM("Error creating directory: " << report_folder_);
      return;
    }
  }

  // Open output file
  std::ofstream output_file(report_folder_ + "/" + report_file_name_);
  if (!output_file.is_open())
  {
    ROS_ERROR_STREAM("Error opening file " << report_file_name_ << " in " << report_folder_);
    return;
  }
  output_file << generateReport();
  output_file.close();
}

/**
 * @brief Replace all instances of a string.
 *
 * @param[in,out] s String to find and replace.
 * @param[in] toReplace String to be replaced.
 * @param[in] replaceWith String to be replaced with.
 */
void replaceAll(std::string& s, const std::string& toReplace, const std::string& replaceWith)
{
  std::string buf;
  std::size_t pos = 0;
  std::size_t prevPos;

  // Reserves rough estimate of final size of string.
  buf.reserve(s.size());

  while (true)
  {
    prevPos = pos;
    pos = s.find(toReplace, pos);
    if (pos == std::string::npos)
      break;
    buf.append(s, prevPos, pos - prevPos);
    buf += replaceWith;
    pos += toReplace.size();
  }

  buf.append(s, prevPos, s.size() - prevPos);
  s.swap(buf);
}

std::string MissionReporterNode::getEndingMessage(const MissionReport::Ptr report) const
{
  std::string output;
  if (report->ending_reason == MissionEndingReasons::Successful)
  {
    output = "Mission completed on " + stampToString(report->end_time.back(), "%Y/%m/%d at %H:%M:%S");
  }
  else if (report->ending_reason == MissionEndingReasons::Disabled)
  {
    output = "Mission disabled on " + stampToString(report->end_time.back(), "%Y/%m/%d at %H:%M:%S");
  }
  else if (report->ending_reason == MissionEndingReasons::Ongoing)
  {
    output = "Ongoing mission.";
  }
  else if (report->ending_reason == MissionEndingReasons::Paused)
  {
    output = "Mission paused on " + stampToString(report->end_time.back(), "%Y/%m/%d at %H:%M:%S");
  }
  else if (report->ending_reason == MissionEndingReasons::Safety ||
           report->ending_reason == MissionEndingReasons::FinishedAfterSafety)
  {
    std::stringstream ss;
    ss << "Mission not completed: ";
    switch (cola2_msgs::RecoveryAction::ABORT)
    {
      case cola2_msgs::RecoveryAction::ABORT:
        ss << "Abort";
        break;
      case cola2_msgs::RecoveryAction::ABORT_AND_SURFACE:
        ss << "Abort and surface";
        break;
      case cola2_msgs::RecoveryAction::EMERGENCY_SURFACE:
        ss << "Emergency surface";
        break;
      case cola2_msgs::RecoveryAction::DROP_WEIGHT:
        ss << "Drop weight";
        break;
      default:
        ss << "Not successful";
        break;
    }
    if (report->safety_msg)
    {
      ss << " on " << stampToString(report->safety_msg->header.stamp, "%Y/%m/%d at %H:%M:%S")
         << ". Reason: " << report->safety_msg->recovery_action.error_string;
    }
    else
    {
      ss << ".";
    }
    output = ss.str();
  }
  return output;
}

std::string MissionReporterNode::generateReport()
{
  if (mission_reports_.size() == 0)
  {
    return "";
  }

  // Set stream ready.
  std::stringstream ss;

  // Report header.
  ss << "# Mission reports " << stampToString(mission_reports_.front()->start_time.front(), "%H:%M:%S %d/%m/%Y");

  // All reports.
  int i = 0;
  for (auto report : mission_reports_)
  {
    // Mission title.
    ss << std::endl << std::endl << "## Mission" << ++i << std::endl << std::endl;

    // Mission name.

    ss << "- Mission name: " << report->mission_name << std::endl;

    // Mission ending
    ss << "- " << getEndingMessage(report) << std::endl;

    // Mission stats
    int32_t total_duration = (report->end_time.back() - report->start_time.front()).sec;
    int32_t total_active = 0;
    for (size_t i = 0; i < report->end_time.size(); ++i)
    {
      total_active += (report->end_time[i] - report->start_time[i]).sec;
    }
    ss << "- Start time: " << stampToString(report->start_time.front(), "%Y/%m/%d %H:%M:%S") << std::endl
       << "- End time: " << stampToString(report->end_time.back(), "%Y/%m/%d %H:%M:%S") << std::endl;
    if (total_duration != total_active)
    {
      ss << "- Total duration: " << durationToHumanReadableString(total_duration) << std::endl
         << "- Active time: " << durationToHumanReadableString(total_active) << std::endl;
    }
    else
    {
      ss << "- Duration: " << durationToHumanReadableString(total_duration) << std::endl;
    }

    ss << std::endl << "|Level|Stamp|Node name|Message|" << std::endl;
    ss << "|---|---|---|---|" << std::endl;
    for (auto msg : report->rosout)
    {
      ss << "|";
      if (msg->level == rosgraph_msgs::Log::DEBUG)
      {
        ss << "DEBUG";
      }
      if (msg->level == rosgraph_msgs::Log::INFO)
      {
        ss << "INFO";
      }
      if (msg->level == rosgraph_msgs::Log::WARN)
      {
        ss << "WARN";
      }
      if (msg->level == rosgraph_msgs::Log::ERROR)
      {
        ss << "ERROR";
      }
      if (msg->level == rosgraph_msgs::Log::FATAL)
      {
        ss << "FATAL";
      }
      ss << "|" << stampToString(msg->header.stamp, rosout_time_format_);
      if (rosout_include_nanoseconds_)
      {
        ss << "." << std::setfill('0') << std::setw(9) << msg->header.stamp.nsec;
      }
      ss << "|" << msg->name;
      std::string message = msg->msg;
      replaceAll(message, "\n", "<br>");
      ss << "|" << message << "|" << std::endl;
    }
  }

  // Publish report as string
  std_msgs::String report_msg;
  report_msg.data = ss.str();

  return report_msg.data;
}

void MissionReporterNode::spin() const
{
  ros::spin();
}

bool MissionReporterNode::getConfig()
{
  bool ok(true);
  std::string reports_folder;
  ok &= cola2::ros::getParam("~reports_folder", reports_folder);
  std::vector<std::string> filter_nodes;
  ok &= cola2::ros::getParam("~filter_nodes", filter_nodes);
  std::vector<std::string> regex_expr_vector;
  ok &= cola2::ros::getParam("~filter_regex", regex_expr_vector);
  bool rosout_include_nanoseconds;
  ok &= cola2::ros::getParam("~rosout_include_nanoseconds", rosout_include_nanoseconds);
  std::string rosout_time_format;
  ok &= cola2::ros::getParam("~rosout_time_format", rosout_time_format);
  if (ok)
  {
    report_folder_ = std::string(getenv("HOME")) + "/" + reports_folder;
    filter_nodes_ = filter_nodes;
    std::string regex_expr("(");
    for (size_t i = 0; i < regex_expr_vector.size(); ++i)
    {
      if (i > 0)
      {
        regex_expr += "|";
      }
      regex_expr += regex_expr_vector[i];
    }
    regex_expr += ")";
    ROS_INFO("Regex expression used: %s", regex_expr.c_str());
    filter_regex_ = std::regex(regex_expr);
    rosout_time_format_ = rosout_time_format;
    rosout_include_nanoseconds_ = rosout_include_nanoseconds;
  }
  return ok;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mission_reporter");
  MissionReporterNode node;
  node.spin();
  return EXIT_SUCCESS;
}
