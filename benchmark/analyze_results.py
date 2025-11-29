"""Analyze benchmark results and generate visualizations"""
import json
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
from typing import List, Dict


class BenchmarkAnalyzer:
    """Analyze and visualize benchmark results"""
    
    def __init__(self, results_file: str):
        with open(results_file, 'r') as f:
            self.data = json.load(f)
        
        # Handle different JSON formats
        if isinstance(self.data, list):
            # Random results format: list of {iteration, config, result}
            self.successful_results = [
                item['result'] for item in self.data if item['result']['success']
            ]
            # Reconstruct stats
            total = len(self.data)
            successful = len(self.successful_results)
            self.data = {
                'stats': {
                    'total_runs': total,
                    'successful_runs': successful,
                    'failed_runs': total - successful,
                    'success_rate': (successful / total * 100) if total > 0 else 0,
                    'avg_total_time': sum(r['total_time'] for r in self.successful_results) / len(self.successful_results) if self.successful_results else 0,
                    'avg_hybrid_distance': sum(r['hybrid_distance'] for r in self.successful_results) / len(self.successful_results) if self.successful_results else 0,
                    'avg_improvement_rrt': sum(r['improvement_over_rrt'] for r in self.successful_results) / len(self.successful_results) if self.successful_results else 0,
                    'avg_improvement_prm': sum(r['improvement_over_prm'] for r in self.successful_results) / len(self.successful_results) if self.successful_results else 0,
                    'min_distance': min(r['hybrid_distance'] for r in self.successful_results) if self.successful_results else 0,
                    'max_distance': max(r['hybrid_distance'] for r in self.successful_results) if self.successful_results else 0,
                    'min_time': min(r['total_time'] for r in self.successful_results) if self.successful_results else 0,
                    'max_time': max(r['total_time'] for r in self.successful_results) if self.successful_results else 0,
                },
                'config': self.data[0]['config'] if self.data else {},
                'results': self.successful_results
            }
        else:
            # Standard benchmark format: {config, stats, results}
            self.successful_results = [
                r for r in self.data['results'] if r['success']
            ]
        
        # Create figures directory if it doesn't exist
        self.figures_dir = Path('figures_base')
        self.figures_dir.mkdir(parents=True, exist_ok=True)
    
    def plot_distance_comparison(self, save_path: str = None):
        """Plot distance comparison between algorithms"""
        if not self.successful_results:
            print("No successful results to plot")
            return
        
        if save_path is None:
            save_path = self.figures_dir / 'distance_comparison.png'
        
        rrt_distances = [r['rrt_distance'] for r in self.successful_results]
        prm_distances = [r['prm_distance'] for r in self.successful_results]
        hybrid_distances = [r['hybrid_distance'] for r in self.successful_results]
        
        fig, ax = plt.subplots(figsize=(12, 6))
        
        x = np.arange(len(self.successful_results))
        width = 0.25
        
        # Draw Hybrid first (in the back), then RRT and PRM on top
        ax.bar(x, hybrid_distances, width, label='Hybrid', alpha=0.8, color='#4CAF50', zorder=1)
        ax.bar(x - width, rrt_distances, width, label='RRT', alpha=0.9, color='#64B5F6', zorder=2)
        ax.bar(x + width, prm_distances, width, label='PRM', alpha=0.9, color='#9400D3', zorder=2)
        
        ax.set_xlabel('Iteration', fontsize=12)
        ax.set_ylabel('Path Distance', fontsize=12)
        ax.set_title('Path Distance Comparison', fontsize=14, fontweight='bold')
        # Reorder legend to match visual importance
        handles, labels = ax.get_legend_handles_labels()
        order = [1, 2, 0]  # RRT, PRM, Hybrid
        ax.legend([handles[i] for i in order], [labels[i] for i in order], fontsize=11)
        ax.grid(True, alpha=0.3, zorder=0)
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"✓ Saved: {save_path}")
        plt.close()
    
    def plot_improvement_distribution(self, save_path: str = None):
        """Plot distribution of improvements"""
        if not self.successful_results:
            print("No successful results to plot")
            return
        
        if save_path is None:
            save_path = self.figures_dir / 'improvement_dist.png'
        
        improvements_rrt = [r['improvement_over_rrt'] for r in self.successful_results]
        improvements_prm = [r['improvement_over_prm'] for r in self.successful_results]
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
        
        # Histogram
        ax1.hist(improvements_rrt, bins=20, alpha=0.7, label='vs RRT', 
                edgecolor='black', color='#64B5F6')
        ax1.hist(improvements_prm, bins=20, alpha=0.7, label='vs PRM', 
                edgecolor='black', color='#9400D3')
        ax1.set_xlabel('Improvement (%)', fontsize=12)
        ax1.set_ylabel('Frequency', fontsize=12)
        ax1.set_title('Improvement Distribution', fontsize=14, fontweight='bold')
        ax1.legend(fontsize=11)
        ax1.grid(True, alpha=0.3)
        
        # Box plot
        bp = ax2.boxplot([improvements_rrt, improvements_prm], 
                         labels=['vs RRT', 'vs PRM'],
                         patch_artist=True)
        bp['boxes'][0].set_facecolor('#64B5F6')
        bp['boxes'][1].set_facecolor('#9400D3')
        ax2.set_ylabel('Improvement (%)', fontsize=12)
        ax2.set_title('Improvement Statistics', fontsize=14, fontweight='bold')
        ax2.grid(True, alpha=0.3, axis='y')
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"✓ Saved: {save_path}")
        plt.close()
    
    def plot_time_analysis(self, save_path: str = None):
        """Plot execution time analysis"""
        if not self.successful_results:
            print("No successful results to plot")
            return
        
        if save_path is None:
            save_path = self.figures_dir / 'time_analysis.png'
        
        times = [r['total_time'] for r in self.successful_results]
        distances = [r['hybrid_distance'] for r in self.successful_results]
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
        
        # Time distribution
        ax1.hist(times, bins=20, edgecolor='black', alpha=0.7, color='#4CAF50')
        ax1.set_xlabel('Total Time (s)', fontsize=12)
        ax1.set_ylabel('Frequency', fontsize=12)
        ax1.set_title('Execution Time Distribution', fontsize=14, fontweight='bold')
        ax1.axvline(np.mean(times), color='red', linestyle='--', 
                   label=f'Mean: {np.mean(times):.2f}s', linewidth=2)
        ax1.legend(fontsize=11)
        ax1.grid(True, alpha=0.3)
        
        # Time vs Distance scatter
        ax2.scatter(times, distances, alpha=0.6, s=50, color='#4CAF50')
        ax2.set_xlabel('Total Time (s)', fontsize=12)
        ax2.set_ylabel('Path Distance', fontsize=12)
        ax2.set_title('Time vs Path Distance', fontsize=14, fontweight='bold')
        
        # Add correlation line
        if len(times) > 1:
            z = np.polyfit(times, distances, 1)
            p = np.poly1d(z)
            ax2.plot(times, p(times), "r--", alpha=0.8, linewidth=2, 
                    label=f'Trend: y={z[0]:.1f}x+{z[1]:.1f}')
            ax2.legend(fontsize=10)
        
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"✓ Saved: {save_path}")
        plt.close()
    
    def plot_success_rate(self, save_path: str = None):
        """Plot success rate visualization"""
        if save_path is None:
            save_path = self.figures_dir / 'success_rate.png'
        
        total = self.data['stats']['total_runs']
        successful = self.data['stats']['successful_runs']
        failed = self.data['stats']['failed_runs']
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
        
        # Pie chart
        colors = ['#4CAF50', '#F44336']
        ax1.pie([successful, failed], labels=['Success', 'Failed'], 
               autopct='%1.1f%%', colors=colors, startangle=90,
               textprops={'fontsize': 12, 'weight': 'bold'})
        ax1.set_title(f'Success Rate ({successful}/{total})', 
                     fontsize=14, fontweight='bold')
        
        # Bar chart with percentages
        categories = ['Success', 'Failed']
        values = [successful, failed]
        bars = ax2.bar(categories, values, color=colors, alpha=0.8, edgecolor='black')
        ax2.set_ylabel('Number of Runs', fontsize=12)
        ax2.set_title('Run Results', fontsize=14, fontweight='bold')
        ax2.grid(True, alpha=0.3, axis='y')
        
        # Add value labels on bars
        for bar in bars:
            height = bar.get_height()
            ax2.text(bar.get_x() + bar.get_width()/2., height,
                    f'{int(height)}',
                    ha='center', va='bottom', fontsize=12, fontweight='bold')
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"✓ Saved: {save_path}")
        plt.close()
    
    def generate_report(self, save_path: str = None):
        """Generate text report"""
        if save_path is None:
            save_path = self.figures_dir / 'benchmark_report.txt'
        
        stats = self.data['stats']
        
        report = []
        report.append("=" * 70)
        report.append("BENCHMARK REPORT")
        report.append("=" * 70)
        report.append(f"\nConfiguration:")
        for key, value in self.data['config'].items():
            report.append(f"  {key:25s}: {value}")
        
        report.append(f"\n{'=' * 70}")
        report.append("Results Summary:")
        report.append(f"  Total iterations:          {stats['total_runs']}")
        report.append(f"  Successful:                {stats['successful_runs']} ({stats['success_rate']:.1f}%)")
        report.append(f"  Failed:                    {stats['failed_runs']}")
        
        if self.successful_results:
            report.append(f"\n{'=' * 70}")
            report.append("Average Performance:")
            report.append(f"  Execution time:            {stats['avg_total_time']:.3f}s")
            report.append(f"  Path distance:             {stats['avg_hybrid_distance']:.1f}")
            report.append(f"  Improvement over RRT:      {stats['avg_improvement_rrt']:.1f}%")
            report.append(f"  Improvement over PRM:      {stats['avg_improvement_prm']:.1f}%")
            
            report.append(f"\n{'=' * 70}")
            report.append("Performance Range:")
            report.append(f"  Distance:                  {stats['min_distance']:.1f} - {stats['max_distance']:.1f}")
            report.append(f"  Time:                      {stats['min_time']:.3f}s - {stats['max_time']:.3f}s")
            
            # Calculate additional statistics
            improvements_rrt = [r['improvement_over_rrt'] for r in self.successful_results]
            improvements_prm = [r['improvement_over_prm'] for r in self.successful_results]
            
            report.append(f"\n{'=' * 70}")
            report.append("Improvement Statistics:")
            report.append(f"  vs RRT - Min/Max/Std:      {min(improvements_rrt):.1f}% / {max(improvements_rrt):.1f}% / {np.std(improvements_rrt):.1f}%")
            report.append(f"  vs PRM - Min/Max/Std:      {min(improvements_prm):.1f}% / {max(improvements_prm):.1f}% / {np.std(improvements_prm):.1f}%")
        
        report.append(f"\n{'=' * 70}")
        
        report_text = "\n".join(report)
        print("\n" + report_text)
        
        with open(save_path, 'w', encoding='utf-8') as f:
            f.write(report_text)
        print(f"\n✓ Saved: {save_path}")
    
    def generate_all_plots(self):
        """Generate all analysis plots and report"""
        print("\n" + "=" * 70)
        print("GENERATING ANALYSIS PLOTS")
        print("=" * 70)
        
        self.plot_distance_comparison()
        self.plot_improvement_distribution()
        self.plot_time_analysis()
        self.plot_success_rate()
        self.generate_report()
        
        print("\n" + "=" * 70)
        print(f"All analysis completed! Check: {self.figures_dir}")
        print("=" * 70)


def main():
    """Main entry point"""
    import sys
    
    results_file = None
    
    if len(sys.argv) >= 2:
        results_file = sys.argv[1]
    else:
        # Auto-find the latest results file
        benchmark_dir = Path('benchmark')
        json_files = list(benchmark_dir.glob('*results*.json'))
        
        if not json_files:
            print("Error: No results files found in benchmark directory")
            print("\nUsage: python analyze_results.py <results_file.json>")
            print("\nExample:")
            print("  python benchmark/analyze_results.py benchmark/benchmark_results_20251126_192133.json")
            return
        
        # Get the most recent file
        results_file = max(json_files, key=lambda p: p.stat().st_mtime)
        print(f"Using latest results file: {results_file}")
    
    if not Path(results_file).exists():
        print(f"Error: File not found: {results_file}")
        return
    
    try:
        analyzer = BenchmarkAnalyzer(results_file)
        analyzer.generate_all_plots()
    except Exception as e:
        print(f"Error during analysis: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()