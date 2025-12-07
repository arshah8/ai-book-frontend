'use client';

import Link from "next/link";
import { useEffect, useState } from "react";
import { isAuthenticated, getUser, signOut } from "@/lib/auth-client";

export default function Home() {
  const [user, setUser] = useState<any>(null);

  useEffect(() => {
    if (isAuthenticated()) {
      setUser(getUser());
    }
  }, []);

  return (
    <div className="min-h-screen bg-gradient-to-br from-blue-50 to-indigo-100 dark:from-slate-900 dark:to-slate-800 transition-colors">
      <div className="container mx-auto px-4 py-16">
        <div className="text-center mb-16">
          <h1 className="text-5xl font-bold text-gray-900 dark:text-white mb-4">
            Physical AI & Humanoid Robotics
          </h1>
          <p className="text-xl text-gray-600 dark:text-gray-300 max-w-2xl mx-auto">
            Bridging the gap between the digital brain and the physical body. 
            Learn to design, simulate, and deploy humanoid robots capable of natural human interactions.
          </p>
        </div>

        <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-8 mb-16">
          <ModuleCard
            number={1}
            title="The Robotic Nervous System"
            subtitle="ROS 2"
            description="Middleware for robot control. Learn ROS 2 Nodes, Topics, Services, and bridging Python Agents to ROS controllers."
            href="/module1"
          />
          <ModuleCard
            number={2}
            title="The Digital Twin"
            subtitle="Gazebo & Unity"
            description="Physics simulation and environment building. Simulate sensors, gravity, and collisions."
            href="/module2"
          />
          <ModuleCard
            number={3}
            title="The AI-Robot Brain"
            subtitle="NVIDIA Isaac™"
            description="Advanced perception and training. Photorealistic simulation, VSLAM, and navigation."
            href="/module3"
          />
          <ModuleCard
            number={4}
            title="Vision-Language-Action"
            subtitle="VLA"
            description="The convergence of LLMs and Robotics. Voice-to-Action, cognitive planning, and natural language control."
            href="/module4"
          />
          <ModuleCard
            number={5}
            title="Capstone Project"
            subtitle="Autonomous Humanoid"
            description="Build a simulated robot that receives voice commands, plans paths, navigates obstacles, and manipulates objects."
            href="/capstone"
          />
        </div>

        <div className="text-center space-y-4">
          <Link
            href="/intro"
            className="inline-block bg-indigo-600 text-white px-8 py-4 rounded-lg text-lg font-semibold hover:bg-indigo-700 transition-colors"
          >
            Start Learning →
          </Link>
        </div>
      </div>
    </div>
  );
}

function ModuleCard({
  number,
  title,
  subtitle,
  description,
  href,
}: {
  number: number;
  title: string;
  subtitle: string;
  description: string;
  href: string;
}) {
  return (
    <Link
      href={href}
      className="block bg-white dark:bg-gray-800 rounded-lg shadow-lg p-6 hover:shadow-xl transition-shadow"
    >
      <div className="flex items-center mb-4">
        <div className="w-10 h-10 mr-3 rounded-full flex items-center justify-center font-bold bg-white text-indigo-700 border border-indigo-200 shadow-sm dark:bg-indigo-600 dark:text-white dark:border-indigo-500">
          {number}
        </div>
        <div>
          <h3 className="text-xl font-bold text-gray-900 dark:text-white">
            {title}
          </h3>
          <p className="text-sm text-indigo-600 dark:text-indigo-400">
            {subtitle}
          </p>
        </div>
      </div>
      <p className="text-gray-600 dark:text-gray-300">{description}</p>
    </Link>
  );
}

