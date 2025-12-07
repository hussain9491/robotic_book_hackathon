'use client';

import React from 'react';

const HomePage = () => {
  return (
    <div className="min-h-screen bg-gradient-to-br from-blue-50 to-indigo-100">
      <header className="bg-white shadow-sm">
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-6">
          <div className="flex justify-between items-center">
            <h1 className="text-3xl font-bold text-gray-900">
              Physical AI & Humanoid Robotics
            </h1>
            <nav>
              <ul className="flex space-x-6">
                <li><a href="#modules" className="text-gray-600 hover:text-indigo-600">Modules</a></li>
                <li><a href="#resources" className="text-gray-600 hover:text-indigo-600">Resources</a></li>
                <li><a href="#about" className="text-gray-600 hover:text-indigo-600">About</a></li>
              </ul>
            </nav>
          </div>
        </div>
      </header>

      <main>
        {/* Hero Section */}
        <section className="py-20">
          <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
            <div className="text-center">
              <h1 className="text-5xl font-extrabold text-gray-900 sm:text-6xl">
                Building Intelligent Humanoid Robots
              </h1>
              <p className="mt-6 text-xl text-gray-600 max-w-3xl mx-auto">
                Learn to develop advanced humanoid robots using ROS 2, AI agents, and modern robotics frameworks.
              </p>
              <div className="mt-10 flex justify-center space-x-4">
                <a
                  href="/docs/module-1-ros2/intro"
                  className="px-6 py-3 border border-transparent text-base font-medium rounded-md text-white bg-indigo-600 shadow-sm hover:bg-indigo-700"
                >
                  Get Started
                </a>
                <a
                  href="/docs/module-1-ros2/chapters/chapter-1-intro"
                  className="px-6 py-3 border border-transparent text-base font-medium rounded-md text-indigo-700 bg-indigo-100 hover:bg-indigo-200"
                >
                  Read Documentation
                </a>
              </div>
            </div>
          </div>
        </section>

        {/* Modules Overview */}
        <section id="modules" className="py-16 bg-white">
          <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
            <div className="text-center">
              <h2 className="text-3xl font-extrabold text-gray-900 sm:text-4xl">
                Course Modules
              </h2>
              <p className="mt-4 text-xl text-gray-600">
                Comprehensive learning path for humanoid robotics
              </p>
            </div>

            <div className="mt-16 grid gap-8 md:grid-cols-2 lg:grid-cols-4">
              {/* Module 1 */}
              <div className="bg-gray-50 rounded-lg p-6 shadow-md hover:shadow-lg transition-shadow">
                <div className="text-indigo-600 text-3xl font-bold mb-2">1</div>
                <h3 className="text-xl font-bold text-gray-900 mb-2">ROS 2 Fundamentals</h3>
                <p className="text-gray-600 mb-4">
                  Learn the robotic nervous system with ROS 2 communication patterns, nodes, topics, and services.
                </p>
                <a href="/docs/module-1-ros2/chapters/chapter-1-intro" className="text-indigo-600 font-medium hover:underline">
                  Explore →
                </a>
              </div>

              {/* Module 2 */}
              <div className="bg-gray-50 rounded-lg p-6 shadow-md hover:shadow-lg transition-shadow">
                <div className="text-indigo-600 text-3xl font-bold mb-2">2</div>
                <h3 className="text-xl font-bold text-gray-900 mb-2">Perception Systems</h3>
                <p className="text-gray-600 mb-4">
                  Vision, sensing, and environment understanding for humanoid robots.
                </p>
                <a href="#" className="text-indigo-600 font-medium hover:underline">
                  Coming Soon →
                </a>
              </div>

              {/* Module 3 */}
              <div className="bg-gray-50 rounded-lg p-6 shadow-md hover:shadow-lg transition-shadow">
                <div className="text-indigo-600 text-3xl font-bold mb-2">3</div>
                <h3 className="text-xl font-bold text-gray-900 mb-2">Motion Planning</h3>
                <p className="text-gray-600 mb-4">
                  Advanced control algorithms and movement planning for humanoid locomotion.
                </p>
                <a href="#" className="text-indigo-600 font-medium hover:underline">
                  Coming Soon →
                </a>
              </div>

              {/* Module 4 */}
              <div className="bg-gray-50 rounded-lg p-6 shadow-md hover:shadow-lg transition-shadow">
                <div className="text-indigo-600 text-3xl font-bold mb-2">4</div>
                <h3 className="text-xl font-bold text-gray-900 mb-2">AI Integration</h3>
                <p className="text-gray-600 mb-4">
                  Machine learning and AI agents for adaptive robotic behaviors.
                </p>
                <a href="/docs/module-1-ros2/chapters/chapter-5-ai-integration" className="text-indigo-600 font-medium hover:underline">
                  Explore →
                </a>
              </div>
            </div>
          </div>
        </section>

        {/* AI Agent Bridge Section */}
        <section className="py-16 bg-gradient-to-r from-indigo-500 to-purple-600 text-white">
          <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
            <div className="md:flex items-center">
              <div className="md:w-1/2 mb-10 md:mb-0">
                <h2 className="text-3xl font-extrabold sm:text-4xl">
                  AI Agent Bridge
                </h2>
                <p className="mt-4 text-lg">
                  Connect AI agents like OpenAI's GPT to your ROS 2 robot systems with our bridge server.
                  Convert natural language commands to robot actions with safety validation.
                </p>
                <div className="mt-6">
                  <h3 className="text-xl font-bold mb-3">Key Features:</h3>
                  <ul className="space-y-2">
                    <li className="flex items-center">
                      <span className="mr-2">✓</span> Natural language to ROS 2 service mapping
                    </li>
                    <li className="flex items-center">
                      <span className="mr-2">✓</span> Safety validation layer
                    </li>
                    <li className="flex items-center">
                      <span className="mr-2">✓</span> Function calling interface
                    </li>
                    <li className="flex items-center">
                      <span className="mr-2">✓</span> 90%+ command accuracy
                    </li>
                  </ul>
                </div>
              </div>
              <div className="md:w-1/2 flex justify-center">
                <div className="bg-white/20 backdrop-blur-sm rounded-xl p-6 w-full max-w-md">
                  <h3 className="text-xl font-bold mb-4">Try it out</h3>
                  <div className="bg-black/20 rounded-lg p-4 font-mono text-sm">
                    <div className="text-green-300">{'> '}{"Move left arm up"}</div>
                    <div className="text-blue-300">{"→ ROS 2: /joint_control/move_to_position"}</div>
                    <div className="text-yellow-300">{"→ Validate safety limits"}</div>
                    <div className="text-green-300">{"→ Execute: left_shoulder_pitch = 0.5 rad"}</div>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </section>

        {/* Resources Section */}
        <section id="resources" className="py-16 bg-gray-50">
          <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
            <div className="text-center">
              <h2 className="text-3xl font-extrabold text-gray-900 sm:text-4xl">
                Learning Resources
              </h2>
              <p className="mt-4 text-xl text-gray-600">
                Tools and examples to accelerate your robotics development
              </p>
            </div>

            <div className="mt-10 grid gap-8 md:grid-cols-3">
              <div className="bg-white rounded-lg p-6 shadow-md">
                <div className="text-indigo-600 text-2xl font-bold mb-3">🤖</div>
                <h3 className="text-xl font-bold text-gray-900 mb-2">Code Examples</h3>
                <p className="text-gray-600 mb-4">
                  Complete ROS 2 nodes, launch files, and URDF models for humanoid robots.
                </p>
                <a href="/examples/ros2-humanoid-baseline" className="text-indigo-600 font-medium hover:underline">
                  Browse Examples →
                </a>
              </div>

              <div className="bg-white rounded-lg p-6 shadow-md">
                <div className="text-indigo-600 text-2xl font-bold mb-3">📚</div>
                <h3 className="text-xl font-bold text-gray-900 mb-2">Documentation</h3>
                <p className="text-gray-600 mb-4">
                  Comprehensive guides for each module with practical exercises.
                </p>
                <a href="/docs" className="text-indigo-600 font-medium hover:underline">
                  Read Docs →
                </a>
              </div>

              <div className="bg-white rounded-lg p-6 shadow-md">
                <div className="text-indigo-600 text-2xl font-bold mb-3">🛠️</div>
                <h3 className="text-xl font-bold text-gray-900 mb-2">Development Tools</h3>
                <p className="text-gray-600 mb-4">
                  RAG chatbot, AI agent bridge, and simulation environments.
                </p>
                <a href="/rag/api" className="text-indigo-600 font-medium hover:underline">
                  Access Tools →
                </a>
              </div>
            </div>
          </div>
        </section>
      </main>

      <footer id="about" className="bg-gray-800 text-white py-12">
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
          <div className="grid grid-cols-1 md:grid-cols-4 gap-8">
            <div>
              <h3 className="text-lg font-bold mb-4">Physical AI & Humanoid Robotics</h3>
              <p className="text-gray-300">
                Building the next generation of intelligent humanoid robots through AI and advanced robotics frameworks.
              </p>
            </div>

            <div>
              <h4 className="text-lg font-bold mb-4">Modules</h4>
              <ul className="space-y-2">
                <li><a href="/docs/module-1-ros2/intro" className="text-gray-300 hover:text-white">ROS 2 Fundamentals</a></li>
                <li><a href="#" className="text-gray-300 hover:text-white">Perception Systems</a></li>
                <li><a href="#" className="text-gray-300 hover:text-white">Motion Planning</a></li>
                <li><a href="#" className="text-gray-300 hover:text-white">AI Integration</a></li>
              </ul>
            </div>

            <div>
              <h4 className="text-lg font-bold mb-4">Resources</h4>
              <ul className="space-y-2">
                <li><a href="/examples" className="text-gray-300 hover:text-white">Code Examples</a></li>
                <li><a href="/specs" className="text-gray-300 hover:text-white">Specifications</a></li>
                <li><a href="/history" className="text-gray-300 hover:text-white">Development History</a></li>
                <li><a href="/rag" className="text-gray-300 hover:text-white">AI Tools</a></li>
              </ul>
            </div>

            <div>
              <h4 className="text-lg font-bold mb-4">Connect</h4>
              <ul className="space-y-2">
                <li><a href="#" className="text-gray-300 hover:text-white">GitHub</a></li>
                <li><a href="#" className="text-gray-300 hover:text-white">Documentation</a></li>
                <li><a href="#" className="text-gray-300 hover:text-white">Community</a></li>
                <li><a href="#" className="text-gray-300 hover:text-white">Support</a></li>
              </ul>
            </div>
          </div>

          <div className="mt-12 pt-8 border-t border-gray-700 text-center text-gray-400">
            <p>© {new Date().getFullYear()} Speckit Robotics. All rights reserved.</p>
          </div>
        </div>
      </footer>
    </div>
  );
};

export default HomePage;