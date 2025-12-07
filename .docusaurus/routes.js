import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '799'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '2c2'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', '005'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'ee0'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '44f'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '6b3'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '5c7'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', 'e5f'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '853'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '96c'),
            routes: [
              {
                path: '/docs/intro',
                component: ComponentCreator('/docs/intro', 'aed'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-ros2/',
                component: ComponentCreator('/docs/module-1-ros2/', '3c4'),
                exact: true
              },
              {
                path: '/docs/module-1-ros2/chapters/chapter-1-intro',
                component: ComponentCreator('/docs/module-1-ros2/chapters/chapter-1-intro', '520'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-ros2/chapters/chapter-2-communication',
                component: ComponentCreator('/docs/module-1-ros2/chapters/chapter-2-communication', '57e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-ros2/chapters/chapter-3-parameters',
                component: ComponentCreator('/docs/module-1-ros2/chapters/chapter-3-parameters', '1b1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-ros2/chapters/chapter-4-urdf',
                component: ComponentCreator('/docs/module-1-ros2/chapters/chapter-4-urdf', 'c58'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-ros2/chapters/chapter-5-ai-integration',
                component: ComponentCreator('/docs/module-1-ros2/chapters/chapter-5-ai-integration', '905'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-ros2/chapters/chapter-6-capstone',
                component: ComponentCreator('/docs/module-1-ros2/chapters/chapter-6-capstone', '03e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-ros2/chapters/cli-basics-tutorial',
                component: ComponentCreator('/docs/module-1-ros2/chapters/cli-basics-tutorial', '7eb'),
                exact: true
              },
              {
                path: '/docs/module-1-ros2/chapters/communication-diagram',
                component: ComponentCreator('/docs/module-1-ros2/chapters/communication-diagram', '99a'),
                exact: true
              },
              {
                path: '/docs/module-1-ros2/chapters/testing-workflow',
                component: ComponentCreator('/docs/module-1-ros2/chapters/testing-workflow', '288'),
                exact: true
              },
              {
                path: '/docs/module-1-ros2/chapters/urdf-visualization-test',
                component: ComponentCreator('/docs/module-1-ros2/chapters/urdf-visualization-test', 'c34'),
                exact: true
              },
              {
                path: '/docs/module-1-ros2/chapters/workspace-diagram',
                component: ComponentCreator('/docs/module-1-ros2/chapters/workspace-diagram', 'a1c'),
                exact: true
              },
              {
                path: '/docs/module-1-ros2/chapters/workspace-setup-guide',
                component: ComponentCreator('/docs/module-1-ros2/chapters/workspace-setup-guide', 'afa'),
                exact: true
              },
              {
                path: '/docs/module-1-ros2/setup-guide',
                component: ComponentCreator('/docs/module-1-ros2/setup-guide', '49d'),
                exact: true
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '95b'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
