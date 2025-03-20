#include "cone_detector/cone_detector.hpp"  // Assicurati che il percorso sia corretto

// costruttore
ConeDetectorNode::ConeDetectorNode() : Node("cone_detector") {
  // Sottoscrizione al topic per le point cloud 2
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/clusters", 10,
    std::bind(&ConeDetectorNode::filter, this, std::placeholders::_1)
  );
  // Publisher per il risultato (vero/falso)
  publisher_ = this->create_publisher<std_msgs::msg::Bool>("cone_detected", 10);

  printf("Cone detector node started.\n");
}

void ConeDetectorNode::filter(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // Conversione del messaggio ROS in point cloud PCL per la manipppolazione 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *cloud); 

  // Segmentazione per il modello di cono
  pcl::SACSegmentation<pcl::PointXYZ> seg; //creo un oggetto per la segmentazione (ricerca del cono)
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_CONE); // imposta il modello di cono
  seg.setMethodType(pcl::SAC_RANSAC);   // imposta il metodo RANSAC
  seg.setDistanceThreshold(0.01);       // imposta la soglia di distanza
  seg.setMaxIterations(1000);           // imposta il numero massimo di iterazioni
 
  // seg.setRadiusLimits(0.06, 0.08);      // imposta i limiti del raggio del cono

  pcl::ModelCoefficients coefficients;  // restituisce la posizione dell'apice, la direzione dell'asse e l'angolo di apertura
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());  //indici dei punti inliers ( che appartengono al modello)
  seg.setInputCloud(cloud);             //imposto la point cloud da analizzare
  seg.segment(*inliers, coefficients);   // segmenta la point cloud

  // Determinazione se il cono è stato rilevato
  bool is_cone = false;
  if (!inliers->indices.empty()) {
    // posso aggiungere una logica per i coefficienti caso troppi falsi positivi
    is_cone = true;  // imposta true se i criteri per un cono sono soddisfatti
  }

  // Pubblica il risultato come Bool
  std_msgs::msg::Bool result_msg;
  result_msg.data = is_cone;
  publisher_->publish(result_msg);
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ConeDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
