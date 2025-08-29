<!-- ERP42 Simulator README (HTML version) -->

<!-- Header -->
<div align="center">
  <h1>🚗 ERP42 Simulator</h1>
  <p><b>Multiple ERP42 Vehicle Simulator (Based on Gazebo)</b></p>

  <!-- Badges (customizable) -->
  <p>
    <img alt="ROS" src="https://img.shields.io/badge/ROS-catkin-blue?logo=ros" />
    <img alt="Gazebo" src="https://img.shields.io/badge/Simulator-Gazebo-lightgrey" />
    <img alt="Linux" src="https://img.shields.io/badge/OS-Ubuntu%20%7C%20Linux-success" />
  </p>

  <!-- Quick links -->
  <p>
    <a href="#build--requirements">Build</a> •
    <a href="#how-to-run">Run</a> •
    <a href="#speed-regulation-by-rank">Speed Regulation</a> •
    <a href="#documentation">Docs</a>
  </p>
</div>

<hr/>

<!-- Build -->
<h2 id="build--requirements">📦 Build &amp; Requirements</h2>

<details open>
  <summary><b>Clone &amp; Initialize Submodules</b></summary>
  <pre><code class="language-bash"># Download and update submodules
cd (your ros workspace)/src
git clone https://github.com/Yonsei-AI-Racing/erp42_simulator.git
cd erp42_simulator
git submodule init
git submodule update</code></pre>
</details>

<details>
  <summary><b>Install Dependencies</b></summary>
  <pre><code class="language-bash"># Install dependencies
rosdep install --from-paths . --ignore-src -r -y</code></pre>
</details>

<details>
  <summary><b>Build Packages</b></summary>
  <pre><code class="language-bash"># Build
cd (your ros workspace)
catkin_make</code></pre>
</details>

<hr/>

<!-- Run -->
<h2 id="how-to-run">▶️ How To Run</h2>

<details open>
  <summary><b>Single Vehicle</b></summary>
  <pre><code class="language-bash">roslaunch erp42_vehicle_gazebo erp42_gazebo_sim_single.launch
roslaunch erp42_vehicle_gazebo erp42_navigation_demo.launch</code></pre>
</details>

<details>
  <summary><b>Multiple Vehicles</b></summary>
  <pre><code class="language-bash">roslaunch erp42_vehicle_gazebo erp42_gazebo_sim_multi.launch
roslaunch erp42_vehicle_gazebo erp42_navigation_multi.launch</code></pre>
</details>

<details>
  <summary><b>Multiple Vehicles + move_base Follower</b></summary>
  <pre><code class="language-bash">roslaunch erp42_vehicle_gazebo erp42_navigation_multi.launch autostart:=true</code></pre>
</details>

<details>
  <summary><b>Multiple Vehicles + move_base Follower (with Input Delay)</b></summary>
  <pre><code class="language-bash">roslaunch erp42_vehicle_gazebo erp42_navigation_multi_w_delay.launch autostart:=true</code></pre>
</details>

<hr/>

<!-- Speed Regulation -->
<h2 id="speed-regulation-by-rank">⚙️ Speed Regulation by Rank</h2>

<p>
  <code>referee_server.py</code> 노드에서 차량 순위에 따라 <b>최대 속도</b>를 제한합니다:
</p>

<ul>
  <li>🥇 <b>1위 차량</b>: <code>5.0 m/s</code></li>
  <li>🥈 <b>2위 이하 차량</b>: <code>4.0 m/s</code></li>
</ul>

<p>해당 파라미터는 다음 네임스페이스에 설정됩니다.</p>

<pre><code class="language-bash">(namespace)/move_base/RegulatedPurePursuitController/max_allowed_velocity</code></pre>

<p>
  이후 <code>regulated_pure_pursuit_controller</code>에서 <b>dynamic_reconfigure</b>로 파라미터가 반영되며,
  최종 속도는
  <a href="https://github.com/Yonsei-AI-Racing/erp42_simulator/blob/4b3ab224bdb26972e309f7cbadde3d039cc45c03/erp42_navigation_demo/regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp#L459">
    <code>applyConstraints</code> 함수의 마지막 부분
  </a>
  에서 결정됩니다.
</p>

<p><b>Note.</b> Planner를 변경하여 사용할 경우, 위 로직이 포함되도록 반드시 반영해 주십시오.</p>

<hr/>

<!-- Docs -->
<h2 id="documentation">📚 Documentation</h2>

<ul>
  <li>🔧 <a href="./docs/troubleshooting_issues.md"><b>ERP42 simulator trouble shooting / issues</b></a></li>
  <li>📂 <a href="./docs/package_tree.md"><b>ERP42 simulator 패키지 구조</b></a></li>
  <li>📘 <a href="./docs/howtouse_gitsubmodule.md"><b>gitsubmodule 사용 가이드</b></a></li>
</ul>

<hr/>

</p>
