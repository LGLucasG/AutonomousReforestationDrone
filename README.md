# Autonomous Reforestation Drone
## *A project by Kariboo*

**Description:** Implementing autonomous behaviours for a drone to be able to reforest any given area without teleoperating.

[![Watch the video](./img/preview.png)](https://youtu.be/yf7zY1jwji8)

[📖 User documentation](docs/user) • [👨‍💻 Developer documentation](docs/developer) • [📈 Project report](docs/report) • [📚 Bibliography](docs/bibliography) • [⚠️ Risk Analysis](docs/risk)

## 📄 This project in short

This project, led by Julien GENESTE, follows on from the [POCDroneReforestation](https://github.com/Kariboo-Corp/POCDroneReforestation) project, in which an **Unmanned Aerial Vehicle** (UAV) equiped with a **seed cannon** was developped. At that point, the drone would only be teleoperated. The video above shows the results of that project, i.e. our starting point.

The idea of this project is to make the drone autonomous using the middleware [ROS 2 (Humble)](https://docs.ros.org/en/humble/index.html). The objectives are as follows:
- Find a way to **precisely locate** the drone at all times during the planting process
- Make the drone **aware of its environment** using the proper sensors. (Is the terrain suitable for planting? Are there obstacles?)
- **Generate a trajectory** to accomplish the mission, ensuring it is as **efficient** as possible.

This project was created in response to the many fires in Gironde and more generally in France.

## 🔍 About this project

|                        |                        |
| :--------------------: | :--------------------: |
| 💼 **Client** | Julien GENESTE |
| 🔒 **Confidentiality** | **Public** |
| 👨‍👨‍👦 **Authors** | [Camille PARRATT]() , [Margo BIRET](https://www.linkedin.com/in/margo-biret/) , [Théodore GIGAULT](https://www.linkedin.com/in/theodoregi/), [Lucas GAVERIAUX](https://www.linkedin.com/in/lucasgaveriaux/) |