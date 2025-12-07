// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1 - The Robotic Nervous System (ROS 2)',
      items: [
        'module-1-ros2/chapters/chapter-1-intro',
        'module-1-ros2/chapters/chapter-2-communication',
        'module-1-ros2/chapters/chapter-3-parameters',
        'module-1-ros2/chapters/chapter-4-urdf',
        'module-1-ros2/chapters/chapter-5-ai-integration',
        'module-1-ros2/chapters/chapter-6-capstone',
      ],
    },
  ],
};

export default sidebars;