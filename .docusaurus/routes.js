import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/docs',
    component: ComponentCreator('/docs', '60c'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '476'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '3fc'),
            routes: [
              {
                path: '/docs/',
                component: ComponentCreator('/docs/', '4a8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2-digital-twin/gazebo-physics',
                component: ComponentCreator('/docs/module2-digital-twin/gazebo-physics', '965'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/module2-digital-twin/gazebo-setup',
                component: ComponentCreator('/docs/module2-digital-twin/gazebo-setup', 'fbd'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/module2-digital-twin/intro',
                component: ComponentCreator('/docs/module2-digital-twin/intro', '434'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/module2-digital-twin/sensor-simulation',
                component: ComponentCreator('/docs/module2-digital-twin/sensor-simulation', 'c76'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/module2-digital-twin/unity-rendering',
                component: ComponentCreator('/docs/module2-digital-twin/unity-rendering', 'd7a'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/module2-digital-twin/urdf-sdf',
                component: ComponentCreator('/docs/module2-digital-twin/urdf-sdf', 'a83'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/module3-isaac/intro',
                component: ComponentCreator('/docs/module3-isaac/intro', '5d9'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/module3-isaac/isaac-ros-vslam',
                component: ComponentCreator('/docs/module3-isaac/isaac-ros-vslam', '634'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/module3-isaac/isaac-sim',
                component: ComponentCreator('/docs/module3-isaac/isaac-sim', '245'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/module3-isaac/nav2-planning',
                component: ComponentCreator('/docs/module3-isaac/nav2-planning', '341'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/module3-isaac/perception-manipulation',
                component: ComponentCreator('/docs/module3-isaac/perception-manipulation', 'a2e'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/module3-isaac/reinforcement-learning',
                component: ComponentCreator('/docs/module3-isaac/reinforcement-learning', '9b5'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/module3-isaac/sim-to-real',
                component: ComponentCreator('/docs/module3-isaac/sim-to-real', '850'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/module3-isaac/synthetic-data',
                component: ComponentCreator('/docs/module3-isaac/synthetic-data', '569'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/module4-vla/capstone-project',
                component: ComponentCreator('/docs/module4-vla/capstone-project', '96a'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/module4-vla/cognitive-planning',
                component: ComponentCreator('/docs/module4-vla/cognitive-planning', 'ce8'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/module4-vla/intro',
                component: ComponentCreator('/docs/module4-vla/intro', '19a'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/module4-vla/multimodal-interaction',
                component: ComponentCreator('/docs/module4-vla/multimodal-interaction', 'b08'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/module4-vla/speech-recognition',
                component: ComponentCreator('/docs/module4-vla/speech-recognition', '223'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/module4-vla/voice-to-action',
                component: ComponentCreator('/docs/module4-vla/voice-to-action', 'c0e'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/modules/ros2-nervous-system/',
                component: ComponentCreator('/docs/modules/ros2-nervous-system/', 'd89'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/modules/ros2-nervous-system/basics/',
                component: ComponentCreator('/docs/modules/ros2-nervous-system/basics/', 'cb3'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/modules/ros2-nervous-system/basics/communication-patterns',
                component: ComponentCreator('/docs/modules/ros2-nervous-system/basics/communication-patterns', '76d'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/modules/ros2-nervous-system/basics/exercises',
                component: ComponentCreator('/docs/modules/ros2-nervous-system/basics/exercises', 'de2'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/modules/ros2-nervous-system/basics/nodes-topics-services',
                component: ComponentCreator('/docs/modules/ros2-nervous-system/basics/nodes-topics-services', '324'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/modules/ros2-nervous-system/humanoid-modeling/',
                component: ComponentCreator('/docs/modules/ros2-nervous-system/humanoid-modeling/', '850'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/modules/ros2-nervous-system/humanoid-modeling/exercises',
                component: ComponentCreator('/docs/modules/ros2-nervous-system/humanoid-modeling/exercises', 'c6e'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/modules/ros2-nervous-system/humanoid-modeling/simulation-setup',
                component: ComponentCreator('/docs/modules/ros2-nervous-system/humanoid-modeling/simulation-setup', '6ed'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/modules/ros2-nervous-system/humanoid-modeling/urdf-structure',
                component: ComponentCreator('/docs/modules/ros2-nervous-system/humanoid-modeling/urdf-structure', 'c14'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/modules/ros2-nervous-system/python-integration/',
                component: ComponentCreator('/docs/modules/ros2-nervous-system/python-integration/', 'f66'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/modules/ros2-nervous-system/python-integration/agent-communication',
                component: ComponentCreator('/docs/modules/ros2-nervous-system/python-integration/agent-communication', '028'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/modules/ros2-nervous-system/python-integration/exercises',
                component: ComponentCreator('/docs/modules/ros2-nervous-system/python-integration/exercises', 'f08'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/modules/ros2-nervous-system/python-integration/rclpy-basics',
                component: ComponentCreator('/docs/modules/ros2-nervous-system/python-integration/rclpy-basics', '2ef'),
                exact: true,
                sidebar: "modules"
              },
              {
                path: '/docs/references/ros2-sources',
                component: ComponentCreator('/docs/references/ros2-sources', 'ee4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/tutorials/simple-robot-demo',
                component: ComponentCreator('/docs/tutorials/simple-robot-demo', '00d'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '2e1'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
