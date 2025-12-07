import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className="hero bg-robotic-gradient text-white py-16 md:py-24 text-center relative overflow-hidden">
      <div className="absolute inset-0" style={{background: "url('data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iNjAiIGhlaWdodD0iNjAiIHZpZXdCb3g9IjAgMCA2MCA2MCIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj48ZyBmaWxsPSJub25lIiBmaWxsLXJ1bGU9ImV2ZW5vZGQiPjxnIGZpbGw9IiNmZmZmZmYiIGZpbGwtb3BhY2l0eT0iMC4wNSI+PGNpcmNsZSBjeD0iMzAiIGN5PSIzMCIgcj0iMiIvPjwvZz48L2c+PC9zdmc+')", opacity: 0.3}}></div>
      <div className="container relative z-10 px-4">
        <h1 className="text-3xl sm:text-4xl md:text-5xl lg:text-6xl font-bold mb-4 text-robotic-cyan leading-tight">
          AI-Native Textbook on Physical AI & Humanoid Robotics
        </h1>
        <p className="text-base sm:text-lg md:text-xl mb-8 text-gray-200 max-w-3xl mx-auto px-4">
          The Complete Practical Guide to Embodied Intelligence, Autonomous Systems, and Human–Robot Collaboration
        </p>
        <div className="flex flex-col sm:flex-row gap-4 justify-center items-center mt-6 px-4">
          <Link
            className="btn btn-primary bg-robotic-cyan hover:bg-robotic-cyan-dark text-white font-bold py-3 px-6 md:py-4 md:px-8 rounded-lg text-base md:text-lg transition-all duration-300 transform hover:scale-105"
            to="/docs/intro">
            Start Reading the Textbook
          </Link>
          <Link
            className="btn btn-secondary bg-transparent border-2 border-robotic-cyan text-robotic-cyan hover:bg-robotic-cyan/10 font-bold py-3 px-6 md:py-4 md:px-8 rounded-lg text-base md:text-lg transition-all duration-300"
            to="/docs">
            View All Modules
          </Link>
        </div>
      </div>
    </header>
  );
}

function TeachingCards() {
  const cards = [
    {
      title: 'Physical AI Fundamentals',
      description: 'Core principles of embodied intelligence, sensorimotor learning, and physics-aware AI systems',
      icon: '🤖'
    },
    {
      title: 'Humanoid Robotics Engineering',
      description: 'Mechanical design, kinematics, dynamics, and control systems for bipedal robots',
      icon: '🦾'
    },
    {
      title: 'Robot Intelligence & Autonomy',
      description: 'Advanced AI techniques for perception, decision-making, and autonomous behavior',
      icon: '🧠'
    },
    {
      title: 'Real-world Projects & Applications',
      description: 'Practical implementations and case studies of deployed robotic systems',
      icon: '🏗️'
    }
  ];

  return (
    <section className="py-16 md:py-24 bg-robotic-dark">
      <div className="container mx-auto px-4">
        <h2 className="text-2xl sm:text-3xl md:text-4xl font-bold text-center mb-10 md:mb-16 text-robotic-cyan">What This Book Teaches</h2>
        <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-6 md:gap-8">
          {cards.map((card, index) => (
            <div key={index} className="card bg-gradient-to-br from-robotic-gray to-robotic-dark rounded-xl p-5 md:p-6 border border-gray-700 hover:border-robotic-cyan transition-all duration-300 transform hover:-translate-y-1 shadow-lg hover:shadow-xl hover:shadow-robotic-cyan/10">
              <div className="text-3xl md:text-4xl text-center mb-3 md:mb-4">{card.icon}</div>
              <div className="text-center">
                <h3 className="text-lg md:text-xl font-bold mb-2 md:mb-3 text-white">{card.title}</h3>
                <p className="text-sm md:text-base text-gray-300">{card.description}</p>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function AINativeFeatures() {
  return (
    <section className="py-16 md:py-24 bg-gray-800">
      <div className="container mx-auto px-4">
        <h2 className="text-2xl sm:text-3xl md:text-4xl font-bold text-center mb-10 md:mb-16 text-robotic-cyan">Why This Textbook is AI-Native</h2>
        <div className="grid grid-cols-1 md:grid-cols-2 gap-8 md:gap-12">
          <div className="bg-gradient-to-br from-robotic-dark to-gray-800 p-6 md:p-8 rounded-xl border border-gray-700">
            <h3 className="text-xl md:text-2xl font-bold mb-3 md:mb-4 text-robotic-cyan">AI-Collaboration Workflow</h3>
            <p className="text-sm md:text-base text-gray-300 leading-relaxed">
              Seamless integration with AI tools for enhanced learning experiences. Students can interact with embedded AI assistants to get real-time help with concepts and code.
            </p>
          </div>
          <div className="bg-gradient-to-br from-robotic-dark to-gray-800 p-6 md:p-8 rounded-xl border border-gray-700">
            <h3 className="text-xl md:text-2xl font-bold mb-3 md:mb-4 text-robotic-cyan">Embedded Agents for Learning</h3>
            <p className="text-sm md:text-base text-gray-300 leading-relaxed">
              Interactive AI agents guide students through complex topics with personalized explanations and adaptive learning paths.
            </p>
          </div>
          <div className="bg-gradient-to-br from-robotic-dark to-gray-800 p-6 md:p-8 rounded-xl border border-gray-700">
            <h3 className="text-xl md:text-2xl font-bold mb-3 md:mb-4 text-robotic-cyan">Adaptive Examples and Code</h3>
            <p className="text-sm md:text-base text-gray-300 leading-relaxed">
              Code examples adapt to student progress and understanding, providing additional context or simplifying complex concepts as needed.
            </p>
          </div>
          <div className="bg-gradient-to-br from-robotic-dark to-gray-800 p-6 md:p-8 rounded-xl border border-gray-700">
            <h3 className="text-xl md:text-2xl font-bold mb-3 md:mb-4 text-robotic-cyan">Practical Robotics Labs</h3>
            <p className="text-sm md:text-base text-gray-300 leading-relaxed">
              Hands-on lab exercises designed to work with AI-assisted debugging and real-time feedback systems.
            </p>
          </div>
        </div>
      </div>
    </section>
  );
}

function FeatureList() {
  const features = [
    '100+ diagrams, workflows, and examples',
    'Hands-on labs for humanoid robotics',
    'End-to-end modules from basics to advanced',
    'RAG-powered embedded AI assistant',
    'Full code for all projects'
  ];

  return (
    <section className="py-16 md:py-24 bg-robotic-dark">
      <div className="container mx-auto px-4">
        <h2 className="text-2xl sm:text-3xl md:text-4xl font-bold text-center mb-10 md:mb-16 text-robotic-cyan">Comprehensive Learning Experience</h2>
        <div className="max-w-3xl md:max-w-4xl mx-auto">
          <ul className="space-y-3 md:space-y-4">
            {features.map((feature, index) => (
              <li key={index} className="flex items-center p-3 md:p-4 bg-robotic-gray rounded-lg border border-gray-700 hover:border-robotic-cyan transition-colors duration-300">
                <div className="w-6 h-6 md:w-8 md:h-8 bg-robotic-cyan rounded-full flex items-center justify-center mr-3 md:mr-4 flex-shrink-0">
                  <span className="text-xs md:text-sm text-white font-bold">✓</span>
                </div>
                <span className="text-sm md:text-lg text-white">{feature}</span>
              </li>
            ))}
          </ul>
        </div>
      </div>
    </section>
  );
}

function CallToAction() {
  return (
    <section className="py-16 md:py-24 bg-robotic-gradient text-white">
      <div className="container mx-auto px-4 text-center">
        <h2 className="text-2xl sm:text-3xl md:text-4xl lg:text-5xl font-bold mb-4 md:mb-6 text-robotic-cyan">Ready to Start Your Journey?</h2>
        <p className="text-base md:text-xl mb-8 md:mb-10 text-gray-200 max-w-xl md:max-w-2xl mx-auto">
          Begin learning about the future of robotics and AI with our comprehensive textbook.
        </p>
        <div className="flex flex-col sm:flex-row gap-4 justify-center items-center">
          <Link
            className="btn btn-primary bg-robotic-cyan hover:bg-robotic-cyan-dark text-white font-bold py-3 px-6 md:py-4 md:px-8 rounded-lg text-base md:text-lg transition-all duration-300 transform hover:scale-105"
            to="/docs/intro">
            Start the Course
          </Link>
          <Link
            className="btn btn-secondary bg-transparent border-2 border-robotic-cyan text-robotic-cyan hover:bg-robotic-cyan/10 font-bold py-3 px-6 md:py-4 md:px-8 rounded-lg text-base md:text-lg transition-all duration-300"
            to="/docs/roadmap">
            View Learning Roadmap
          </Link>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home`}
      description="AI-Native Textbook on Physical AI & Humanoid Robotics - The Complete Practical Guide to Embodied Intelligence, Autonomous Systems, and Human–Robot Collaboration">
      <HomepageHeader />
      <main>
        <TeachingCards />
        <AINativeFeatures />
        <FeatureList />
        <CallToAction />
      </main>
    </Layout>
  );
}