import Link from "next/link";
import Chatbot from "@/components/Chatbot";

export default function Module2Page() {
  return (
    <div className="min-h-screen bg-white dark:bg-gray-900">
      <div className="container mx-auto px-4 py-12 max-w-4xl">
        <Link
          href="/"
          className="text-indigo-600 hover:text-indigo-800 mb-8 inline-block"
        >
          ← Back to Home
        </Link>

        <article className="prose prose-lg dark:prose-invert max-w-none">
          <h1>Module 2: The Digital Twin (Gazebo & Unity)</h1>
          
          <section className="my-8">
            <h2>Introduction</h2>
            <p>
              Before deploying robots in the real world, we need to test and validate our 
              algorithms in simulation. This module covers physics simulation with Gazebo 
              and high-fidelity rendering with Unity.
            </p>
          </section>

          <section className="my-8">
            <h2>Gazebo Simulation</h2>
            <p>
              Gazebo is a powerful physics simulation environment that allows you to test 
              robot behaviors in realistic virtual environments. It simulates physics, 
              gravity, collisions, and various sensors.
            </p>
            <h3>Key Features</h3>
            <ul>
              <li>Physics engine (ODE, Bullet, Simbody)</li>
              <li>Sensor simulation (LiDAR, cameras, IMUs)</li>
              <li>Plugin system for custom behaviors</li>
              <li>Integration with ROS 2</li>
            </ul>
          </section>

          <section className="my-8">
            <h2>Physics Simulation</h2>
            <p>
              Gazebo uses physics engines to simulate realistic interactions. This includes:
            </p>
            <ul>
              <li><strong>Gravity:</strong> Objects fall and respond to gravitational forces</li>
              <li><strong>Collisions:</strong> Realistic collision detection and response</li>
              <li><strong>Friction:</strong> Surface interactions affect movement</li>
              <li><strong>Dynamics:</strong> Mass, inertia, and momentum calculations</li>
            </ul>
          </section>

          <section className="my-8">
            <h2>Unity for High-Fidelity Rendering</h2>
            <p>
              While Gazebo excels at physics simulation, Unity provides photorealistic 
              rendering capabilities. Unity is particularly useful for:
            </p>
            <ul>
              <li>Human-robot interaction scenarios</li>
              <li>Training computer vision models with synthetic data</li>
              <li>Creating immersive virtual environments</li>
              <li>Testing perception algorithms</li>
            </ul>
          </section>

          <section className="my-8">
            <h2>Sensor Simulation</h2>
            <p>
              Simulating sensors accurately is crucial for developing perception algorithms:
            </p>
            <ul>
              <li><strong>LiDAR:</strong> 3D point cloud generation</li>
              <li><strong>Depth Cameras:</strong> RGB-D data simulation</li>
              <li><strong>IMUs:</strong> Accelerometer and gyroscope data</li>
              <li><strong>Cameras:</strong> RGB image generation with realistic noise</li>
            </ul>
          </section>
        </article>

        <div className="mt-12 flex gap-4">
          <Link
            href="/module1"
            className="bg-gray-600 text-white px-6 py-3 rounded-lg font-semibold hover:bg-gray-700 transition-colors"
          >
            ← Previous
          </Link>
          <Link
            href="/module3"
            className="bg-indigo-600 text-white px-6 py-3 rounded-lg font-semibold hover:bg-indigo-700 transition-colors"
          >
            Next: Module 3 →
          </Link>
        </div>
      </div>

      <Chatbot />
    </div>
  );
}

