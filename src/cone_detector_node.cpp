#include "cone_detector/cone_detector.hpp" // Assicurati che il percorso sia corretto

// costruttore
ConeDetectorNode::ConeDetectorNode() : Node("cone_detector")
{
  // Sottoscrizione al topic per le point cloud 2
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/clusters", 10,
      std::bind(&ConeDetectorNode::filter, this, std::placeholders::_1));
  // Publisher per il risultato (vero/falso)
  publisher_ = this->create_publisher<std_msgs::msg::Bool>("cone_detected", 10);

  printf("Cone detector node started.\n");
}

void ConeDetectorNode::filter(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Se il numero di punti nella point cloud è inferiore a 10, esce dalla funzione
  if (msg->width * msg->height < 10)
  {
    RCLCPP_WARN(this->get_logger(), "PointCloud troppo piccola, ignorata.");
    return;
  }
  // Conversione del messaggio ROS in point cloud PCL per la manipppolazione
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *cloud);

  // calcolo delle normali
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;                                   // oggetto per il calcolo delle normali
  ne.setInputCloud(cloud);                                                                // imposto la point cloud per il calcolo
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>()); // creo un oggetto per la ricerca dei punti vicini
  ne.setSearchMethod(tree);                                                               // imposto il metodo di ricerca su ne
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());    // creo un oggetto per contenere le normali
  ne.setKSearch(30);                                                                      // Numero di vicini usati per stimare la normale
  ne.compute(*cloud_normals);                                                             // calcolo delle normali

  // Segmentazione per il modello di cono
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg; // creo un oggetto per la segmentazione basat sulle normali (ricerca del cono)
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_CONE); // imposta il modello di cono
  seg.setMethodType(pcl::SAC_RANSAC);   // metodo RANSAC
  seg.setDistanceThreshold(0.03);       // soglia di distanza
  seg.setMaxIterations(1000);           // numero massimo di iterazioni
  seg.setNormalDistanceWeight(0.05);    // Peso della distanza rispetto alle normali

  // seg.setRadiusLimits(0.06, 0.08);      // imposta i limiti del raggio del cono

  // Imposta input cloud e input normals
  seg.setInputCloud(cloud);
  seg.setInputNormals(cloud_normals);

  // Segmentazione
  pcl::ModelCoefficients coefficients;                     // restituisce la posizione dell'apice, la direzione dell'asse e l'angolo di apertura
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices()); // indici dei punti inliers ( che appartengono al modello)
  seg.segment(*inliers, coefficients);                     // segmenta la point cloud

  // Determinazione se il cono è stato rilevato
  bool is_cone = false;
  if (!inliers->indices.empty())
  {
    std::cout << "Cone detected with " << inliers->indices.size() << " inliers." << std::endl; // debug

    // logica per i coefficienti per eliminare falsi positivi
    if (coefficients.values.size() > 4)
    {
      // posizione apice
      if (coefficients.values.size() > 0)
      {
        double apex_z = coefficients.values[0]; // altezza dell'apice del cono

        if (apex_z < 3.0 || apex_z > 4.0) // altezza dell'apice del cono tra 3.0 e 4.0
        {
          std::cout << "Scartato: apice del cono fuori dall'intervallo atteso (Z = " << apex_z << ")." << std::endl;
          is_cone = false;
        }
        else
          std::cout << "Apice del cono a Z = " << apex_z << std::endl; // debug
        is_cone = true;
      }

      // angolo apertura cono
      double opening_angle = std::abs(coefficients.values[4]); // in radianti
      double min_angle = 5.0 * M_PI / 180.0;                   // imposto angolo minimo e massimo per il cono tra 5 e 20
      double max_angle = 20.0 * M_PI / 180.0;

      if (opening_angle < min_angle || opening_angle > max_angle)
      {
        std::cout << "Scartato: angolo non compatibile con un cono (" << opening_angle * 180.0 / M_PI << "°)." << std::endl;
        is_cone = false;
      }
      else
        std::cout << "Angolo di apertura del cono: " << opening_angle * 180.0 / M_PI << "°" << std::endl; // debug
      is_cone = true;
    }
    if (is_cone == true)
    {
      // Pubblica il risultato come Bool
      std_msgs::msg::Bool result_msg;
      result_msg.data = is_cone;
      publisher_->publish(result_msg);
      std::cout << "Cone detected." << std::endl;
    }
  }
  else
    std::cout << "No cone detected." << std::endl;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ConeDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
