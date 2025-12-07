import Link from "next/link";
import Chatbot from "@/components/Chatbot";

export default function Module1Page() {
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
          <h1>Module 1: The Robotic Nervous System (ROS 2)</h1>
          
          <section className="my-8">
            <h2>Introduction</h2>
            <p>
              ROS 2 (Robot Operating System 2) is the middleware that enables communication 
              between different components of a robot system. Think of it as the nervous system 
              that allows the robot's "brain" (AI algorithms) to communicate with its "body" 
              (sensors and actuators).
            </p>
          </section>

          <section className="my-8">
            <h2>ROS 2 Architecture</h2>
            <p>
              ROS 2 follows a distributed architecture where different processes (nodes) 
              communicate through topics, services, and actions. This design allows for 
              modularity and scalability.
            </p>
            <h3>Core Concepts</h3>
            <ul>
              <li><strong>Nodes:</strong> Individual processes that perform specific tasks</li>
              <li><strong>Topics:</strong> Asynchronous communication channels for streaming data</li>
              <li><strong>Services:</strong> Synchronous request-response communication</li>
              <li><strong>Actions:</strong> Long-running tasks with feedback</li>
            </ul>
          </section>

          <section className="my-8">
            <h2>ROS 2 Nodes, Topics, and Services</h2>
            <p>
              Nodes are the fundamental building blocks of ROS 2. Each node is responsible 
              for a specific function, such as reading sensor data, processing images, or 
              controlling motors.
            </p>
            <pre className="bg-gray-100 dark:bg-gray-800 p-4 rounded">
{`# Example: Creating a ROS 2 Node in Python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.publisher = self.create_publisher(
            String, 'topic_name', 10
        )
        self.subscription = self.create_subscription(
            String, 'input_topic', self.callback, 10
        )
    
    def callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()`}
            </pre>
          </section>

          <section className="my-8">
            <h2>Bridging Python Agents to ROS Controllers</h2>
            <p>
              Modern AI agents written in Python can be integrated with ROS 2 using the 
              rclpy library. This allows you to leverage AI models (like GPT, vision models, 
              etc.) to control robots.
            </p>
            <p>
              The key is to create ROS 2 nodes that wrap your AI agent logic, allowing the 
              agent to subscribe to sensor data and publish control commands.
            </p>
          </section>

          <section className="my-8">
            <h2>Understanding URDF for Humanoids</h2>
            <p>
              URDF (Unified Robot Description Format) is an XML format used to describe the 
              physical structure of a robot, including its links (parts), joints, and their 
              relationships.
            </p>
            <p>
              For humanoid robots, URDF files define the kinematic chain from the base 
              (pelvis) through the legs, torso, arms, and head. This description is essential 
              for simulation, motion planning, and control.
            </p>
          </section>
        </article>

        <div className="mt-12 flex gap-4">
          <Link
            href="/intro"
            className="bg-gray-600 text-white px-6 py-3 rounded-lg font-semibold hover:bg-gray-700 transition-colors"
          >
            ← Previous
          </Link>
          <Link
            href="/module2"
            className="bg-indigo-600 text-white px-6 py-3 rounded-lg font-semibold hover:bg-indigo-700 transition-colors"
          >
            Next: Module 2 →
          </Link>
        </div>
      </div>

      <Chatbot />
    </div>
  );
}

