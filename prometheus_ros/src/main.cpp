#include <prometheus/counter.h>
#include <prometheus/exposer.h>
#include <prometheus/gauge.h>
#include <prometheus/registry.h>
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

int main()
{
  std::cout << __FILE__ << "," << __LINE__ << std::endl;
  // Prometheusのレジストリとゲージを作成
  std::shared_ptr<prometheus::Registry> registry = std::make_shared<prometheus::Registry>();
  std::cout << __FILE__ << "," << __LINE__ << std::endl;

  auto & gauge_family = prometheus::BuildGauge()
                          .Name("cpu_usage_percent")
                          .Help("CPU usage in percent")
                          .Register(*registry);

  std::cout << __FILE__ << "," << __LINE__ << std::endl;
  auto & cpu_usage_gauge = gauge_family.Add({{"type", "total"}});
  std::cout << __FILE__ << "," << __LINE__ << std::endl;

  // Exposerを使ってHTTPエンドポイントでメトリクスを公開
  prometheus::Exposer exposer{"0.0.0.0:9100"};
  std::cout << __FILE__ << "," << __LINE__ << std::endl;
  exposer.RegisterCollectable(registry);
  std::cout << __FILE__ << "," << __LINE__ << std::endl;

  // 定期的にCPU使用率を更新
  while (true) {
    std::cout << __FILE__ << "," << __LINE__ << std::endl;
    cpu_usage_gauge.Set(1);
    std::cout << __FILE__ << "," << __LINE__ << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}