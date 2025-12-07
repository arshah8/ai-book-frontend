/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module1/introduction',
        'module1/architecture',
        'module1/nodes-topics-services',
        'module1/urdf',
        'module1/python-agents',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module2/introduction',
        'module2/gazebo-simulation',
        'module2/unity-rendering',
        'module2/sensors',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module3/introduction',
        'module3/isaac-sim',
        'module3/isaac-ros',
        'module3/nav2',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module4/introduction',
        'module4/voice-to-action',
        'module4/cognitive-planning',
        'module4/llm-integration',
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: [
        'capstone/overview',
        'capstone/implementation',
        'capstone/deployment',
      ],
    },
    {
      type: 'category',
      label: 'Course Information',
      items: [
        'course/overview',
        'course/learning-outcomes',
        'course/hardware-requirements',
        'course/weekly-breakdown',
        'course/assessments',
      ],
    },
  ],
};

module.exports = sidebars;

