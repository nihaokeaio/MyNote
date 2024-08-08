#pragma once
#include <string>
#include <vector>
using namespace std;


struct ListNode {
	int val;
	ListNode* next;
	ListNode() : val(0), next(nullptr) {}
	ListNode(int x) : val(x), next(nullptr) {}
	ListNode(int x, ListNode* next) : val(x), next(next) {}
};

struct TreeNode
{
	int val;
	TreeNode* left;
	TreeNode* right;
	TreeNode():val(0),left(nullptr),right(nullptr){}
	TreeNode(int x) :val(x), left(nullptr), right(nullptr) {}
};

class MyAlgorithms
{
public:
	///字符串匹配算法
	//\字符串匹配算法:从txt从寻找pat，并返回下标，若没有匹配则返回-1
	//展示普通的搜索算法
	int serachStr(const string& txt, const string& pat);

	//\KMP状态机算法
	//\文档链接:https://zhuanlan.zhihu.com/p/83334559
	int serachStrKMP(const string& txt, const string& pat);
	void createKMPVec(vector<vector<int>>& nums, const string& pat);




	///链表相关
	///反转链表
	ListNode* reverseList(ListNode* head);

	///删除链表的倒数第N个节点
	ListNode* removeNthFromEnd(ListNode* head, int n);




	///二叉树
	///递归遍历
	
private:
};
